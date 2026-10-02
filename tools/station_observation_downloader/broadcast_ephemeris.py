"""Fetch a daily RINEX-3 mixed broadcast navigation file, with validated reuse."""
from datetime import datetime
import gzip
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
from urllib.parse import urlparse

import requests


def filename(day):
    return f'BRDC00IGS_R_{day:%Y%j}0000_01D_MN.rnx'


def inspect_navigation(path, day):
    """Validate the header, complete navigation records and the requested date.

    Broadcast epochs use each constellation's time scale. This verifies records
    for the selected calendar date, without treating them as 1 Hz observations.
    """
    records = matching = 0
    systems = set()
    with path.open(encoding='ascii') as source:
        header = source.readline()
        if (header[60:].strip() != 'RINEX VERSION / TYPE' or
                not 3 <= float(header[:9]) < 4 or header[20:21] != 'N' or header[40:41] != 'M'):
            raise ValueError('不是 RINEX 3 混合广播星历文件')
        for line in source:
            if line[60:].strip() == 'END OF HEADER':
                break
        else:
            raise ValueError('广播星历头部不完整')
        for line in source:
            if not re.match(r'^[GRECJSI]\d{2} ', line):
                raise ValueError('广播星历记录格式错误或文件被截断')
            values = line[3:23].split()
            if len(values) != 6:
                raise ValueError('广播星历历元格式错误')
            epoch = datetime(*map(int, values))
            system = line[0]
            if len(line.rstrip('\r\n')) < 80:
                raise ValueError('广播星历钟差记录不完整')
            rows = 4 if line[0] in 'RS' else 8
            for index in range(rows):
                if index:
                    line = source.readline()
                    if not line.startswith('    ') or len(line.rstrip('\r\n')) < 23:
                        raise ValueError('广播星历记录不完整')
                fields = line[23:] if index == 0 else line[4:]
                for offset in range(0, len(fields.rstrip('\r\n')), 19):
                    value = fields[offset:offset+19].strip()
                    if value and not math.isfinite(float(value.replace('D','E').replace('d','e'))):
                        raise ValueError('广播星历含无效数值')
            systems.add(system)
            records += 1
            matching += int(epoch.date() == day)
    if not matching:
        raise ValueError(f'广播星历没有 {day} 当天的有效记录')
    return {'date':str(day), 'records':records, 'records_on_date':matching, 'systems':sorted(systems)}


def ensure_broadcast(day, destination, client, log):
    name = filename(day)
    target = destination/name
    legacy = destination/f'brdc{day:%j}0.{day.year%100:02d}p'/name
    for candidate in (target, legacy):
        if candidate.is_file():
            try:
                info = inspect_navigation(candidate, day)
            except (OSError, ValueError, UnicodeError, IndexError):
                log(f'已有广播星历未通过校验，将补充下载：{candidate.name}')
            else:
                log(f'复用已有当天广播星历，不重复下载：{candidate}')
                return {**info, 'path':str(candidate), 'file':name, 'downloaded':0, 'source':'local'}
    if client.offline:
        raise RuntimeError('离线模式缺少有效的当天广播星历；取消星历勾选或取消离线模式后重试')
    urls = [
        ('BKG', f'https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{day.year}/{day:%j}/{name}.gz'),
        ('CDDIS', f'https://cddis.nasa.gov/archive/gnss/data/daily/{day.year}/{day:%j}/{day.year%100:02d}p/{name}.gz'),
    ]
    errors = []
    for provider, url in urls:
        log(f'正在下载 {day} 全天混合广播星历（{provider}）…')
        # No Earthdata credentials in the public mirror session. Preserve proxy settings.
        with requests.Session() as public:
            public.trust_env = False
            public.proxies.update(requests.utils.get_environ_proxies(url))
            for attempt in range(3):
                try:
                    with tempfile.TemporaryDirectory(prefix='.broadcast-', dir=destination) as folder:
                        compressed = Path(folder)/(name+'.gz')
                        temporary = Path(folder)/name
                        response = (client.response(url, stream=True) if provider == 'CDDIS'
                                    else public.get(url, timeout=(15,60), stream=True))
                        with response, compressed.open('wb') as output:
                            response.raise_for_status()
                            if urlparse(response.url).hostname not in ('igs.bkg.bund.de','cddis.nasa.gov'):
                                raise ValueError('广播星历下载重定向到非数据服务器')
                            for chunk in response.iter_content(1024*1024):
                                output.write(chunk)
                        # Read to EOF to verify the gzip checksum before publication.
                        with gzip.open(compressed,'rb') as source, temporary.open('wb') as output:
                            shutil.copyfileobj(source,output,1024*1024)
                        info = inspect_navigation(temporary,day)
                        os.replace(temporary,target)
                    log(f'当天广播星历已下载并校验：{target}')
                    return {**info,'path':str(target),'file':name,'downloaded':1,'source':provider,'url':url}
                except (requests.RequestException, EOFError, OSError, ValueError, UnicodeError, IndexError) as error:
                    if isinstance(error, requests.RequestException) and attempt < 2:
                        log(f'{provider} 星历下载连接中断，重试 {attempt+1}/2')
                        time.sleep(attempt+1)
                        continue
                    errors.append(f'{provider}: {error}')
                    break
    raise RuntimeError('当天广播星历下载未完成；已有观测结果保留。' + '；'.join(errors))
