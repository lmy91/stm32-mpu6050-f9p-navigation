"""输入流动站观测文件，默认下载覆盖时段的最少分段；全天观测为可选项。

python wuh2_download_merge.py "C:/data/rover.obs"
python wuh2_download_merge.py "C:/data/rover.obs" --offline

仅在需要访问服务器时询问 Earthdata 账号。密码不写入文件。
依赖 requests，以及本机 CRX2RNX 和 GFZRNX；工具路径可以用参数指定。
"""
import argparse
from contextlib import contextmanager
from datetime import date, datetime, timedelta
import getpass
import gzip
import hashlib
from html.parser import HTMLParser
import json
import netrc
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
from urllib.parse import urljoin, urlparse

import requests
from broadcast_ephemeris import ensure_broadcast

SCRIPT_DIR = Path(__file__).resolve().parent
AUTH_HOSTS = {'cddis.nasa.gov', 'urs.earthdata.nasa.gov'}
NO_WINDOW = getattr(subprocess, 'CREATE_NO_WINDOW', 0)


@contextmanager
def output_lock(path):
    """OS releases the lock on exit or crash; a stale lock file never blocks retry."""
    with path.open('a+b') as handle:
        if handle.tell()==0:
            handle.write(b'0')
            handle.flush()
        handle.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as error:
            raise RuntimeError('此保存位置已有任务在运行，请等待它结束后重试') from error
        try:
            yield
        finally:
            handle.seek(0)
            if os.name=='nt':
                msvcrt.locking(handle.fileno(),msvcrt.LK_UNLCK,1)
            else:
                fcntl.flock(handle.fileno(),fcntl.LOCK_UN)


def say(message):
    print(message, flush=True)


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.part')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def load_json(path, default=None):
    if not path.exists():
        return {} if default is None else default
    return json.loads(path.read_text(encoding='utf-8'))


def fingerprint(path):
    stat = path.stat()
    return {'size':stat.st_size, 'mtime_ns':stat.st_mtime_ns}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def check_gzip(path):
    # Reading to EOF checks both gzip CRC and truncation, without retaining data in RAM.
    with gzip.open(path, 'rb') as source:
        while source.read(1024*1024):
            pass


def epoch_from_fields(fields):
    return datetime(*map(int, fields[:5])) + timedelta(seconds=float(fields[5]))


def inspect_rinex(path, station):
    marker = interval = declared_first = declared_last = time_system = None
    first = last = None
    count = 0
    gaps = []
    header_done = False
    with path.open(encoding='ascii') as source:
        for line in source:
            label = line[60:].strip()
            if label == 'MARKER NAME':
                marker = line[:60].strip()
            elif label == 'INTERVAL':
                interval = float(line[:10])
            elif label in ('TIME OF FIRST OBS', 'TIME OF LAST OBS'):
                fields = line[:60].split()
                value = epoch_from_fields(fields)
                if label == 'TIME OF FIRST OBS':
                    declared_first = value
                    time_system = fields[6] if len(fields)>6 else 'GPS'
                else:
                    declared_last = value
            elif label == 'END OF HEADER':
                header_done = True
                break
        for line in source:
            if not line.startswith('>'):
                continue
            fields = line.split()
            if int(fields[7]) not in (0, 1):
                continue
            current = epoch_from_fields(fields[1:7])
            if last is not None:
                step = (current-last).total_seconds()
                if step<=0:
                    raise ValueError(f'{path.name}: 历元重复或时间倒序')
                if abs(step-1)>1e-6:
                    gaps.append({'from':last.isoformat(), 'to':current.isoformat(), 'step_s':step})
            first = first or current
            last = current
            count += 1
    if not header_done or not marker or marker.upper() not in (station,station[:4]):
        raise ValueError(f'{path.name}: 不完整的头部或测站不匹配')
    if interval!=1 or count==0 or time_system not in ('GPS',None):
        raise ValueError(f'{path.name}: 不是有效的 GPS 时间、1秒观测文件')
    if declared_first is not None and first!=declared_first:
        raise ValueError(f'{path.name}: 首历元与头部不符')
    if declared_last is not None and last!=declared_last:
        raise ValueError(f'{path.name}: 尾历元与头部不符，可能截断')
    return {'file':path.name,'marker':marker,'interval_s':interval,'epochs':count,
            'first_gps':first.isoformat(),'last_gps':last.isoformat(),'gaps':gaps}


def rover_times(path, cancelled=None):
    values = {}
    with path.open(encoding='ascii') as source:
        version = source.readline()
        if (version[60:].strip()!='RINEX VERSION / TYPE' or
            not 3 <= float(version[:9]) < 4 or version[20:21]!='O'):
            raise ValueError('请选择已解码的 RINEX 3 观测文件；不支持 UBX、导航文件或压缩文件')
        header_done = False
        for line in source:
            if cancelled and cancelled():
                raise InterruptedError('已停止读取流动站文件')
            label = line[60:].strip()
            if label in ('TIME OF FIRST OBS','TIME OF LAST OBS'):
                fields = line[:60].split()
                if len(fields)>6 and fields[6]!='GPS':
                    raise ValueError('当前自动时段定位要求 rover.obs 使用 GPS 时间')
                values[label] = epoch_from_fields(fields)
            if label == 'END OF HEADER':
                header_done = True
                break
        if not header_done:
            raise ValueError('流动站观测文件缺少完整的 RINEX 头部')
        if 'TIME OF FIRST OBS' not in values or 'TIME OF LAST OBS' not in values:
            first = last = None
            for line in source:
                if cancelled and cancelled():
                    raise InterruptedError('已停止读取流动站文件')
                if line.startswith('>'):
                    fields = line.split()
                    if int(fields[7]) in (0,1):
                        current = epoch_from_fields(fields[1:7])
                        first = first or current
                        last = current
            if first is None:
                raise ValueError('无法获取流动站首尾观测时间，请提供有效的 RINEX 3 观测文件')
            values['TIME OF FIRST OBS'] = first
            values['TIME OF LAST OBS'] = last
    return values['TIME OF FIRST OBS'],values['TIME OF LAST OBS']


def quarter_floor(value):
    return value.replace(minute=(value.minute//15)*15,second=0,microsecond=0)


def required_slots(start, end, full_day=False):
    begin = datetime.combine(start.date(),datetime.min.time()) if full_day else quarter_floor(start)
    last = begin+timedelta(hours=23,minutes=45) if full_day else quarter_floor(end)
    result = []
    while begin<=last:
        result.append(begin.strftime('%H%M'))
        begin += timedelta(minutes=15)
    return result


class EarthdataSession(requests.Session):
    def rebuild_auth(self, prepared_request, response):
        if urlparse(prepared_request.url).hostname not in AUTH_HOSTS:
            prepared_request.headers.pop('Authorization',None)
        elif self.auth:
            prepared_request.prepare_auth(self.auth)
        else:
            super().rebuild_auth(prepared_request,response)


class Client:
    def __init__(self, offline=False, username=None, non_interactive=False):
        self.offline = offline
        self.username = username
        self.session = None
        self.downloads = 0
        self.non_interactive = non_interactive

    def authenticate(self):
        if self.session is not None:
            return
        if self.offline:
            raise RuntimeError('离线模式缺少必要文件；请取消 --offline 后补齐')
        username = self.username or os.environ.get('EARTHDATA_USERNAME')
        password = os.environ.get('EARTHDATA_PASSWORD')
        if not (username and password):
            for candidate in (Path.home()/'.netrc',Path.home()/'_netrc'):
                if candidate.exists():
                    try:
                        saved = netrc.netrc(str(candidate)).authenticators('urs.earthdata.nasa.gov')
                        if saved and (not username or username==saved[0]):
                            username,password = saved[0],saved[2]
                            break
                    except (OSError,netrc.NetrcParseError):
                        pass
        if self.non_interactive and not (username and password):
            raise PermissionError('需要下载数据：请在界面填写 Earthdata 用户名和密码后重试；首次使用请在 Earthdata 网站授权 CDDIS 应用')
        username = username or input('Earthdata 用户名: ').strip()
        password = password or getpass.getpass('Earthdata 密码（隐藏输入）: ')
        if not username or not password:
            raise ValueError('账号和密码不能为空')
        self.session = EarthdataSession()
        self.session.auth = (username,password)

    def response(self, url, stream=False):
        if urlparse(url).hostname!='cddis.nasa.gov':
            raise ValueError('仅支持 CDDIS 的下载地址')
        self.authenticate()
        response = self.session.get(url,timeout=(15,60),stream=stream)
        if urlparse(response.url).hostname=='urs.earthdata.nasa.gov' or response.status_code in (401,403):
            response.close()
            raise PermissionError('Earthdata 登录失败；请检查账号或首次使用的 CDDIS 应用授权')
        if response.status_code==404:
            response.close()
            raise FileNotFoundError(f'服务器不存在该目录或文件: {url}')
        response.raise_for_status()
        return response

    def listing(self, url):
        for attempt in range(5):
            try:
                with self.response(url) as response:
                    return response.text
            except requests.RequestException:
                if attempt==4:
                    raise
                time.sleep(attempt+1)

    def download(self, url, target):
        temporary = target.with_name(target.name+'.part')
        for attempt in range(5):
            try:
                with self.response(url,stream=True) as response, temporary.open('wb') as output:
                    for chunk in response.iter_content(1024*1024):
                        output.write(chunk)
                check_gzip(temporary)
                os.replace(temporary,target)
                self.downloads += 1
                return
            except (requests.RequestException,EOFError,gzip.BadGzipFile):
                if attempt==4:
                    raise
                say(f'连接或压缩流中断，重试 {attempt+1}/4: {target.name}')
                time.sleep(attempt+1)


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.urls = []

    def handle_starttag(self, tag, attrs):
        if tag.lower()=='a':
            self.urls.extend(value.strip() for key,value in attrs if key.lower()=='href' and value)


def tool_path(explicit, variable, name, candidates):
    choices = [explicit,os.environ.get(variable),shutil.which(name),
               SCRIPT_DIR/'bin'/name, SCRIPT_DIR/'bin'/(name+'.exe'), *candidates]
    return next((str(Path(p).resolve()) for p in choices if p and Path(p).is_file()),None)


def ensure_rinex(url, destination, station, client, state, old_hashes, converter):
    name = urlparse(url).path.rsplit('/',1)[-1]
    compressed = destination/'compressed'/name
    rinex = destination/'rinex'/name[:-3].replace('.crx','.rnx')
    previous = state['parts'].get(name,{})
    if not compressed.exists() and rinex.exists():
        try:
            info = inspect_rinex(rinex,station)
        except (ValueError,UnicodeError,IndexError):
            pass
        else:
            return rinex,info,True
    expected_hash = previous.get('sha256') or old_hashes.get(name)
    valid_compressed = False
    if compressed.exists():
        actual_hash = sha256(compressed)
        if expected_hash:
            valid_compressed = actual_hash==expected_hash
        else:
            try:
                check_gzip(compressed)
                valid_compressed = True
            except (EOFError,OSError):
                pass
    if not valid_compressed:
        if compressed.exists():
            backup = compressed.with_name(compressed.name+f'.invalid.{time.time_ns()}')
            compressed.rename(backup)
            say(f'保留损坏文件备份: {backup.name}')
        client.download(url,compressed)
        actual_hash = sha256(compressed)
    # Cached validation is only reused when both source hash and output fingerprint match.
    cached = previous.get('rinex')
    if (cached and rinex.exists() and previous.get('sha256')==actual_hash and
            previous.get('rinex_fingerprint')==fingerprint(rinex)):
        info = cached
    else:
        info = None
        if rinex.exists() and valid_compressed:
            try:
                info = inspect_rinex(rinex,station)
            except (ValueError,UnicodeError,IndexError):
                say(f'转换文件不完整，将重新生成: {rinex.name}')
        if info is None:
            with tempfile.TemporaryDirectory(prefix='.convert-',dir=destination) as temporary_dir:
                temporary_dir = Path(temporary_dir)
                unpacked = temporary_dir/name[:-3]
                with gzip.open(compressed,'rb') as source,unpacked.open('wb') as output:
                    shutil.copyfileobj(source,output,1024*1024)
                converted = temporary_dir/rinex.name
                if unpacked.suffix.lower()=='.crx':
                    if not converter:
                        raise RuntimeError('未找到 CRX2RNX；请用 --crx2rnx 指定程序路径')
                    with converted.open('wb') as output:
                        result = subprocess.run([converter,str(unpacked),'-'],stdout=output,stderr=subprocess.PIPE,creationflags=NO_WINDOW)
                    if result.returncode!=0:
                        raise RuntimeError('CRX2RNX 转换失败: '+result.stderr.decode(errors='replace'))
                else:
                    converted = unpacked
                info = inspect_rinex(converted,station)
                os.replace(converted,rinex)
    state['parts'][name] = {'url':url,'sha256':actual_hash,'bytes':compressed.stat().st_size,
                            'rinex_fingerprint':fingerprint(rinex),'rinex':info}
    return rinex,info,valid_compressed


def merge(files, infos, output, station, state, merger):
    identity = [{'name':path.name,**fingerprint(path)} for path in files]
    previous = state['merges'].get(output.name,{})
    if (output.exists() and previous.get('inputs')==identity and
            previous.get('output_fingerprint')==fingerprint(output)):
        say(f'跳过已生成合并文件: {output.name}')
        return previous['info']
    expected_count = sum(info['epochs'] for info in infos)
    def compatible(info):
        return (info['epochs']==expected_count and info['first_gps']==infos[0]['first_gps'] and
                info['last_gps']==infos[-1]['last_gps'])
    info = None
    # Adopt pre-existing outputs from the earlier download, after actual validation.
    if output.exists() and not previous and output.stat().st_mtime_ns>=max(p.stat().st_mtime_ns for p in files):
        try:
            candidate = inspect_rinex(output,station)
            if compatible(candidate):
                info = candidate
                say(f'复用已验证合并文件: {output.name}')
        except (ValueError,UnicodeError,IndexError):
            pass
    if info is None:
        if not merger:
            raise RuntimeError('未找到 GFZRNX；请用 --gfzrnx 指定程序路径')
        say(f'正在合并 {len(files)} 个文件: {output.name}')
        day = datetime.fromisoformat(infos[0]['first_gps']).date()
        work = output.parent if output.parent.name==f'.wuh2_work_{station}_{day:%Y%m%d}' else output.parent/f'.wuh2_work_{station}_{day:%Y%m%d}'
        work.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.merge-',dir=work) as temporary_dir:
            temporary = Path(temporary_dir)/output.name
            options = ['-splice_direct'] if len(files)>8 else []
            result = subprocess.run([merger,'-finp',*[str(p) for p in files],'-fout',str(temporary),'-vo','3',*options],
                                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,creationflags=NO_WINDOW)
            output.with_suffix('.merge.log').write_bytes(result.stdout+result.stderr)
            if result.returncode!=0 or not temporary.exists():
                raise RuntimeError(f'GFZRNX 合并失败，详见 {output.with_suffix(".merge.log").name}')
            info = inspect_rinex(temporary,station)
            if not compatible(info):
                raise ValueError('合并结果与输入历元数量/时段不一致；保留已有输出')
            os.replace(temporary,output)
    state['merges'][output.name] = {'inputs':identity,'output_fingerprint':fingerprint(output),'info':info}
    return info


def header_time(value, label):
    fields = (f'{value.year:6d}{value.month:6d}{value.day:6d}'
              f'{value.hour:6d}{value.minute:6d}{value.second+value.microsecond/1e6:13.7f}')
    return fields.ljust(48)+'GPS'.ljust(12)+label+'\n'


class NativeCropRequired(Exception):
    pass


def crop_stream(source_path, output_path, start, end):
    """Copy complete RINEX-3 epochs; preserve observation values and update the header."""
    with source_path.open(encoding='ascii') as source, output_path.open('w',encoding='ascii',newline='\n') as target:
        header = []
        for line in source:
            if line[60:].strip()=='END OF HEADER':
                break
            header.append(line)
        else:
            raise ValueError('输入观测文件缺少 END OF HEADER')
        omit = {'TIME OF FIRST OBS','TIME OF LAST OBS','PRN / # OF OBS','# OF SATELLITES'}
        for line in header:
            if line[60:].strip() not in omit:
                target.write(line)
        target.write(header_time(start,'TIME OF FIRST OBS'))
        target.write(header_time(end,'TIME OF LAST OBS'))
        target.write('Subset selected using rover observation GPS time'.ljust(60)+'COMMENT\n')
        target.write(''.ljust(60)+'END OF HEADER\n')
        copying = False
        for line in source:
            if line.startswith('>'):
                fields = line.split()
                current = epoch_from_fields(fields[1:7])
                flag = int(fields[7])
                if current>end:
                    break
                if flag not in (0,1):
                    # Earlier receiver/header events change the effective header.
                    raise NativeCropRequired()
                copying = start<=current<=end
            if copying:
                target.write(line)


def make_subset(daily, output, start, end, station, state, merger):
    identity = {'daily':fingerprint(daily),'start':start.isoformat(),'end':end.isoformat()}
    previous = state.setdefault('subsets',{}).get(output.name,{})
    if (output.exists() and previous.get('input')==identity and
            previous.get('output_fingerprint')==fingerprint(output)):
        say(f'跳过已生成时段文件: {output.name}')
        return previous['info']
    say(f'按流动站时间提取: {start} ～ {end} (GPS)')
    work = output.parent/f'.wuh2_work_{station}_{start:%Y%m%d}'
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.subset-',dir=work) as temporary_dir:
        temporary = Path(temporary_dir)/output.name
        try:
            crop_stream(daily,temporary,start,end)
        except NativeCropRequired:
            if not merger:
                raise RuntimeError('文件含前置接收机事件，需要用 --gfzrnx 指定 GFZRNX')
            temporary.unlink(missing_ok=True)
            duration = int((end-start).total_seconds())+1
            result = subprocess.run([merger,'-finp',str(daily),'-fout',str(temporary),
                                     '-vo','3','-epo_beg',start.strftime('%Y-%m-%d_%H%M%S'),
                                     '-d',str(duration)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,creationflags=NO_WINDOW)
            if result.returncode!=0:
                raise RuntimeError('时段提取失败: '+result.stderr.decode(errors='replace'))
        info = inspect_rinex(temporary,station)
        if info['first_gps']!=start.isoformat() or info['last_gps']!=end.isoformat() or info['gaps']:
            raise ValueError('时段文件没有完整覆盖请求范围；保留原文件及中间数据')
        os.replace(temporary,output)
    state['subsets'][output.name] = {'input':identity,'output_fingerprint':fingerprint(output),'info':info}
    return info


def cleanup_intermediate(destination, final_files, station, day):
    """Remove only this pipeline's known intermediates after both outputs are verified."""
    resolved = destination.resolve()
    folders = [destination/f'.wuh2_work_{station}_{day:%Y%m%d}']
    for folder in folders:
        if folder.exists():
            if folder.resolve().parent!=resolved or folder.is_symlink():
                raise ValueError(f'拒绝清理指向输出目录之外的路径: {folder}')
            shutil.rmtree(folder)
    keep = {path.resolve() for path in final_files}
    for path in destination.iterdir():
        if not path.is_file() or path.resolve() in keep:
            continue
        is_old_output = (path.suffix.lower()=='.rnx' and (
            path.name.startswith(f'{station}_R_{day:%Y%j}') or
            path.name.startswith(f'{station[:4]}_{day:%Y%m%d}_')))
        is_merge_log = path.name.startswith(f'{station}_R_{day:%Y%j}') and path.name.endswith('.merge.log')
        if is_merge_log or is_old_output:
            path.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('rover',type=Path,help='流动站 RINEX 3 观测文件')
    parser.add_argument('--station',default='WUH200CHN',help='默认基站 WUH200CHN')
    parser.add_argument('--output',type=Path,help='默认流动站观测文件所在文件夹')
    parser.add_argument('--offline',action='store_true',help='禁止网络访问，仅复用已有数据')
    parser.add_argument('--username',help='Earthdata 用户名；密码隐藏输入')
    parser.add_argument('--crx2rnx',help='CRX2RNX 可执行文件路径')
    parser.add_argument('--gfzrnx',help='GFZRNX 可执行文件路径')
    parser.add_argument('--broadcast-ephemeris',action='store_true',help='同时下载流动站日期对应的全天混合广播星历')
    parser.add_argument('--full-day',action='store_true',help='下载全天96个分段；默认仅下载覆盖流动站时段的最少分段')
    parser.add_argument('--non-interactive',action='store_true',help='不提示交互输入账号，供界面调用')
    parser.add_argument('--json-summary',action='store_true',help='在日志末尾输出供界面读取的结果')
    args = parser.parse_args(argv)
    rover_path = args.rover.resolve()
    rover_first,rover_last = rover_times(rover_path)
    if rover_last<rover_first or rover_first.date()!=rover_last.date():
        parser.error('流动站首尾观测必须在同一个 GPS 日期内；跨日数据请先按天分割')
    day = rover_first.date()
    start = rover_first.replace(microsecond=0)
    end = rover_last.replace(microsecond=0)+timedelta(seconds=int(rover_last.microsecond>0))
    if end.date()!=day:
        parser.error('末历元需要次日基站历元，单日文件无法包围该时间范围')
    station = args.station.upper()
    if not re.fullmatch(r'[A-Z0-9]{9}',station):
        parser.error('--station 要使用九位长站名，例如 WUH200CHN')
    destination = (args.output or rover_path.parent).resolve()
    if destination==rover_path:
        parser.error('输出路径须为文件夹')
    destination.mkdir(parents=True,exist_ok=True)
    state_dir = (destination if destination==rover_path.parent else destination.parent)/'.wuh2_pipeline'
    state_dir.mkdir(exist_ok=True)
    state_file = state_dir/(destination.name+'_'+hashlib.sha256(str(destination).encode()).hexdigest()[:12]+'.json')
    with output_lock(state_file.with_suffix('.lock')):
        return generate_outputs(args,rover_path,rover_first,rover_last,station,destination,state_file)


def generate_outputs(args,rover_path,rover_first,rover_last,station,destination,state_file):
    day = rover_first.date()
    start = rover_first.replace(microsecond=0)
    end = rover_last.replace(microsecond=0)+timedelta(seconds=int(rover_last.microsecond>0))
    slots = required_slots(start,end,args.full_day)
    say(f'{"全天模式" if args.full_day else "时段模式"}：需要 {len(slots)} 个15分钟分段')
    state = load_json(state_file,{'parts':{},'merges':{},'subsets':{}})
    daily_path = destination/f'{station}_R_{day:%Y%j}0000_01D_01S_MO.rnx'
    subset_path = destination/f'{station[:4]}_{day:%Y%m%d}_{start:%H%M%S}_{end:%H%M%S}_1s.rnx'
    client = Client(args.offline,args.username,args.non_interactive)
    merger = tool_path(args.gfzrnx,'GFZRNX','gfzrnx',[
        r'D:\Dr\algorithm\FAST-main\fast\bin\gfzrnx_win.exe',
        r'D:\Dr\algorithm\fast\win_bin\bin\gfzrnx_win.exe'])
    subset_info = None
    def covers(info):
        return (info['first_gps']==start.isoformat() and info['last_gps']==end.isoformat() and
                info['epochs']==int((end-start).total_seconds())+1 and not info['gaps'])
    if not args.full_day and subset_path.exists():
        previous = state.get('sessions',{}).get(subset_path.name,{})
        old = state.get('subsets',{}).get(subset_path.name,{})
        try:
            if previous.get('fingerprint')==fingerprint(subset_path):
                candidate = previous['info']
            elif old.get('output_fingerprint')==fingerprint(subset_path):
                candidate = old['info']
            else:
                candidate = inspect_rinex(subset_path,station)
            if covers(candidate):
                subset_info = candidate
                say('复用已有匹配时段文件，不访问观测下载服务器')
        except (ValueError,UnicodeError,IndexError,KeyError):
            pass
    daily_info = None
    begin = datetime.combine(day,datetime.min.time())
    previous = state.get('daily',{})
    if daily_path.exists():
        if previous.get('fingerprint')==fingerprint(daily_path):
            daily_info = previous['info']
        elif args.full_day or subset_info is None:
            say('检查已有全天文件，检查通过后不访问下载服务器…')
            try:
                candidate = inspect_rinex(daily_path,station)
                if (candidate['epochs']==86400 and not candidate['gaps'] and
                    candidate['first_gps']==begin.isoformat() and
                    candidate['last_gps']==(begin+timedelta(days=1,seconds=-1)).isoformat()):
                    daily_info = candidate
            except (ValueError,UnicodeError,IndexError):
                pass
    if daily_info is not None:
        state['daily'] = {'fingerprint':fingerprint(daily_path),'info':daily_info}
        say('复用已有完整全天文件，不重复下载')
        if subset_info is None:
            subset_info = make_subset(daily_path,subset_path,start,end,station,state,merger)
    elif subset_info is None or args.full_day:
        work_dir = destination/f'.wuh2_work_{station}_{day:%Y%m%d}'
        for name in ('compressed','rinex'):
            (work_dir/name).mkdir(parents=True,exist_ok=True)
        base = f'https://cddis.nasa.gov/archive/gnss/data/highrate/{day.year}/{day:%j}/{day.year%100:02d}d/'
        pattern = re.compile(rf'{station}_[RS]_{day:%Y%j}(\d{{4}})_15M_01S_MO\.(crx|rnx)\.gz',re.I)
        urls = list(state.get('archive_urls',[]))
        urls.extend(load_json(destination/'archive_listing_day.json').get('urls',[]))
        urls.extend(part['url'] for part in state['parts'].values() if 'url' in part)
        for folder in ('compressed','rinex'):
            for path in (work_dir/folder).iterdir():
                name = path.name if folder=='compressed' else path.name+'.gz'
                match = pattern.fullmatch(name)
                if match:
                    urls.append(base+match[1][:2]+'/'+name)
        def choose(candidates):
            result = {}
            for url in sorted(set(candidates)):
                match = pattern.fullmatch(urlparse(url).path.rsplit('/',1)[-1])
                if url.startswith(base) and match and match[1] in slots:
                    result.setdefault(match[1],url)
            return result
        chosen = choose(urls)
        missing_hours = sorted({key[:2] for key in slots if key not in chosen})
        for hour in missing_hours:
            listing = Links()
            listing.feed(client.listing(base+hour+'/'))
            urls.extend(urljoin(base+hour+'/',href) for href in listing.urls)
            say(f'已读取必要服务器目录：{hour}时')
        chosen = choose(urls)
        state['archive_urls'] = sorted({url for url in urls if url.startswith(base) and
                                      pattern.fullmatch(urlparse(url).path.rsplit('/',1)[-1])})
        atomic_json(state_file,state)
        if len(chosen)!=len(slots):
            missing = ', '.join(key for key in slots if key not in chosen)
            raise FileNotFoundError(f'缺少覆盖流动站所需分段：{missing}；保留已下载文件')
        if len(slots)>1 and not merger:
            raise RuntimeError('未找到 GFZRNX，请在高级选项选择程序，或用 --gfzrnx 指定；不会开始下载')
        old_manifest = load_json(destination/'download_manifest.json')
        old_hashes = {Path(entry['compressed_file']).name:entry['sha256'] for entry in old_manifest.get('files',[])}
        converter = tool_path(args.crx2rnx,'CRX2RNX','crx2rnx',[
            r'D:\Dr\algorithm\RTKLIB_EX_2.5.0\crx2rnx.exe',
            r'D:\Dr\algorithm\FAST-main\fast\bin\crx2rnx.exe'])
        files,infos = [],[]
        for index,key in enumerate(slots,1):
            path,info,reused = ensure_rinex(chosen[key],work_dir,station,client,state,old_hashes,converter)
            expected = begin+timedelta(hours=int(key[:2]),minutes=int(key[2:]))
            if (info['epochs']!=900 or info['gaps'] or info['first_gps']!=expected.isoformat() or
                info['last_gps']!=(expected+timedelta(seconds=899)).isoformat()):
                raise ValueError(f'{key} 分段不完整；保留中间数据，无法完整覆盖请求范围')
            files.append(path)
            infos.append(info)
            atomic_json(state_file,state)
            say(f'[{index}/{len(slots)}] {"复用" if reused else "下载"}: {path.name}')
        if args.full_day:
            daily_info = merge(files,infos,daily_path,station,state,merger)
            if daily_info['gaps'] or daily_info['epochs']!=86400:
                raise ValueError('全天文件存在缺口，保留中间数据')
            state['daily'] = {'fingerprint':fingerprint(daily_path),'info':daily_info}
            atomic_json(state_file,state)
            source_path = daily_path
        elif len(files)==1:
            source_path = files[0]
        else:
            source_path = work_dir/f'{station}_R_{day:%Y%j}{slots[0]}_coverage.rnx'
            merge(files,infos,source_path,station,state,merger)
        subset_info = make_subset(source_path,subset_path,start,end,station,state,merger)
    state.setdefault('sessions',{})[subset_path.name] = {'fingerprint':fingerprint(subset_path),'info':subset_info}
    summary = {'station':station,'date':str(day),'rover':str(rover_path),
               'rover_first_gps':rover_first.isoformat(),'rover_last_gps':rover_last.isoformat(),
               'mode':'full_day' if args.full_day else 'session','full_day_requested':args.full_day,
               'required_segments':len(slots),'downloaded_this_run':client.downloads,
               'whole_day_complete':daily_info is not None,'daily':daily_info,
               'session':subset_info,'session_covers_rover':True,'output_directory':str(destination)}
    state['last_run'] = summary
    atomic_json(state_file,state)
    # Keep pre-existing daily data even when the full-day option is unchecked.
    cleanup_intermediate(destination,[daily_path,subset_path,rover_path],station,day)
    if args.broadcast_ephemeris:
        summary['broadcast'] = ensure_broadcast(day,destination,client,say)
        state['last_run'] = summary
        atomic_json(state_file,state)
    say(f'完成：本次下载 {client.downloads} 个观测分段；匹配时段文件：')
    say(str(subset_path))
    if args.full_day:
        say('全天观测：'+str(daily_path))
    if args.broadcast_ephemeris:
        say('广播星历：'+summary['broadcast']['path'])
    if args.json_summary:
        say('WUH2_RESULT_JSON='+json.dumps(summary,ensure_ascii=False))
    return summary


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    try:
        main()
    except (OSError,ValueError,RuntimeError,requests.RequestException,KeyError,IndexError) as error:
        say(f'未完成: {error}')
        raise SystemExit(1)
