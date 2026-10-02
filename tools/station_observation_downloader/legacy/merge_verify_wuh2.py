"""Merge downloaded WUH2 1 Hz RINEX files and check whole-day/session coverage."""
from datetime import datetime, timedelta
import json
from pathlib import Path
import subprocess
from download_wuh2_1s import inspect_rinex

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'WUH2_20260923_1s'
GFZRNX = Path(r'D:\Dr\algorithm\FAST-main\fast\bin\gfzrnx_win.exe')


def merge(files, output):
    if not files:
        raise ValueError('No downloaded RINEX observation files to merge')
    if output.exists():
        existing = inspect_rinex(output)
        inputs = [inspect_rinex(path) for path in files]
        if (existing['epochs']==sum(part['epochs'] for part in inputs) and
                existing['first_gps']==inputs[0]['first_gps'] and
                existing['last_gps']==inputs[-1]['last_gps']):
            return existing
        raise FileExistsError(f'Existing output differs from inputs; preserve or rename it: {output.name}')
    # The whole day is large; direct splicing avoids holding all observations in RAM.
    splice_options = ['-splice_direct'] if len(files)>8 else []
    completed = subprocess.run([str(GFZRNX),'-finp',*[str(p) for p in files],
                                '-fout',str(output),'-vo','3',*splice_options], capture_output=True)
    log = completed.stdout + completed.stderr
    output.with_suffix('.merge.log').write_bytes(log)
    if completed.returncode != 0 or not output.exists():
        raise RuntimeError(f'GFZRNX merge failed; inspect {output.with_suffix(".merge.log").name}')
    return inspect_rinex(output)


def read_rover_times():
    first = last = None
    with (ROOT/'rover.obs').open(encoding='ascii') as source:
        for line in source:
            label = line[60:].strip()
            if label in ('TIME OF FIRST OBS', 'TIME OF LAST OBS'):
                fields = line[:60].split()
                value = datetime(*map(int,fields[:5]))+timedelta(seconds=float(fields[5]))
                if label == 'TIME OF FIRST OBS':
                    first = value
                else:
                    last = value
            if label == 'END OF HEADER':
                break
    assert first is not None and last is not None
    return first,last


def main():
    files = sorted((BASE/'rinex').glob('WUH200CHN_*_2026266*_15M_01S_MO.rnx'))
    daily = BASE/'WUH200CHN_R_20262660000_01D_01S_MO.rnx'
    daily_info = merge(files,daily)
    # These two actual archived blocks cover the rover record with margins.
    selected = [p for p in files if '20262660245_' in p.name or '20262660300_' in p.name]
    session = BASE/'WUH200CHN_R_20262660245_30M_01S_MO.rnx'
    session_info = merge(selected,session)
    rover_first,rover_last = read_rover_times()
    session_first = datetime.fromisoformat(session_info['first_gps'])
    session_last = datetime.fromisoformat(session_info['last_gps'])
    covered = session_first<=rover_first and session_last>=rover_last
    intersects_gap = any(datetime.fromisoformat(gap['from'])<rover_last and
                         datetime.fromisoformat(gap['to'])>rover_first
                         for gap in session_info['gaps'])
    expected_first = datetime(2026,9,23)
    expected_last = datetime(2026,9,23,23,59,59)
    complete_day = (len(files)==96 and daily_info['epochs']==86400 and
                    daily_info['first_gps']==expected_first.isoformat() and
                    daily_info['last_gps']==expected_last.isoformat() and
                    not daily_info['gaps'])
    result = {'station':'WUH200CHN','date':'2026-09-23','interval_s':1,
              'downloaded_blocks':len(files),'expected_blocks':96,
              'whole_day_complete':complete_day,'daily':daily_info,
              'session':session_info,'rover_first_gps':rover_first.isoformat(),
              'rover_last_gps':rover_last.isoformat(),
              'session_covers_rover':covered,'gap_intersects_rover':intersects_gap}
    (BASE/'verification_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    (BASE/'下载状态.txt').write_text(
        f'已下载 {len(files)} 个15分钟WUH2真实1秒观测文件。\n'
        f'全天历元：{daily_info["epochs"]}；全天完整：{complete_day}。\n'
        f'本次采集覆盖：{covered}；采集时段内缺口：{intersects_gap}。\n'
        f'全天合并：{daily.name}\n采集两段合并：{session.name}\n'
        '详细统计：verification_summary.json\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('daily','session')},ensure_ascii=False,indent=2))
    print('Daily epochs:',daily_info['epochs'],'gaps:',len(daily_info['gaps']))
    print('Session epochs:',session_info['epochs'],'gaps:',len(session_info['gaps']))


if __name__ == '__main__':
    main()
