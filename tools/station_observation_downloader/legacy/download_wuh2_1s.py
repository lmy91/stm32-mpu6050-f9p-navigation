"""Download WUH2 1 Hz data from CDDIS after local Earthdata authentication.

Passwords are requested locally without echo, and are never written to disk.
Default: full GPS date 2026-09-23. --session-only: the 02:45 and 03:00 blocks.
--probe: check the archive access without prompting for credentials.
"""
import argparse
from datetime import date, datetime, timedelta
import getpass
import gzip
import hashlib
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from urllib.parse import urljoin, urlparse

import requests

ROOT = Path(__file__).resolve().parent
DEST = ROOT / 'WUH2_20260923_1s'
DATE = date(2026, 9, 23)
DOY = DATE.timetuple().tm_yday
BASE = f'https://cddis.nasa.gov/archive/gnss/data/highrate/{DATE.year}/{DOY:03d}/{DATE.year % 100:02d}d/'
ALLOWED = {'cddis.nasa.gov', 'urs.earthdata.nasa.gov'}


class EarthdataSession(requests.Session):
    def rebuild_auth(self, prepared_request, response):
        # Earthdata uses a redirect from the archive to its login service.
        # Credentials must never be forwarded to any other hostname.
        if urlparse(prepared_request.url).hostname not in ALLOWED:
            prepared_request.headers.pop('Authorization', None)
            return
        if self.auth:
            prepared_request.prepare_auth(self.auth)
        else:
            super().rebuild_auth(prepared_request, response)


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.hrefs = []

    def handle_starttag(self, tag, attrs):
        if tag.lower() == 'a':
            self.hrefs.extend(v.strip() for k, v in attrs if k.lower() == 'href' and v)


def checked_get(session, url):
    assert urlparse(url).hostname == 'cddis.nasa.gov'
    for attempt in range(5):
        try:
            response = session.get(url, timeout=(15, 45))
            if urlparse(response.url).hostname == 'urs.earthdata.nasa.gov':
                raise PermissionError('Earthdata login is required or was not accepted; no GNSS file downloaded.')
            if response.status_code in (401, 403):
                raise PermissionError(f'CDDIS denied access (HTTP {response.status_code}).')
            if response.status_code == 404:
                return None
            response.raise_for_status()
            return response
        except requests.RequestException:
            if attempt == 4:
                raise
            time.sleep(attempt + 1)


def find_converter():
    candidates = [os.environ.get('CRX2RNX'), shutil.which('crx2rnx'),
                  r'D:\Dr\algorithm\RTKLIB_EX_2.5.0\crx2rnx.exe',
                  r'D:\Dr\algorithm\FAST-main\fast\bin\crx2rnx.exe']
    return next((str(p) for p in candidates if p and Path(p).is_file()), None)


def inspect_rinex(path):
    interval = None
    marker = None
    obs_types = {}
    last_system = None
    first = last = None
    count = 0
    gaps = []
    with path.open(encoding='ascii', errors='strict') as source:
        for line in source:
            label = line[60:].strip()
            if label == 'MARKER NAME':
                marker = line[:60].strip()
            elif label == 'INTERVAL':
                interval = float(line[:10])
            elif label == 'SYS / # / OBS TYPES':
                if line[0].strip():
                    last_system = line[0]
                if last_system is not None:
                    obs_types.setdefault(last_system, []).extend(line[7:60].split())
            elif label == 'END OF HEADER':
                break
        for line in source:
            if not line.startswith('>'):
                continue
            fields = line.split()
            if int(fields[7]) not in (0, 1):
                continue
            epoch = datetime(*map(int, fields[1:6])) + timedelta(seconds=float(fields[6]))
            if last is not None:
                step = (epoch-last).total_seconds()
                if step != 1:
                    gaps.append({'from':last.isoformat(), 'to':epoch.isoformat(), 'step_s':step})
            first = first or epoch
            last = epoch
            count += 1
    if interval != 1 or count == 0:
        raise ValueError(f'{path.name}: not verified as a nonempty 1-second observation file')
    if not marker or 'WUH2' not in marker.upper():
        raise ValueError(f'{path.name}: station marker is not WUH2')
    return {'file':path.name, 'marker':marker, 'interval_s':interval, 'epochs':count,
            'first_gps':first.isoformat(), 'last_gps':last.isoformat(),
            'gaps':gaps, 'observation_types':obs_types}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-only', action='store_true')
    parser.add_argument('--probe', action='store_true')
    args = parser.parse_args()
    DEST.mkdir(parents=True, exist_ok=True)
    session = EarthdataSession()
    if not args.probe:
        username = os.environ.get('EARTHDATA_USERNAME') or input('Earthdata username: ').strip()
        password = os.environ.get('EARTHDATA_PASSWORD') or getpass.getpass('Earthdata password (hidden): ')
        if not username or not password:
            raise ValueError('An Earthdata username and password are required.')
        session.auth = (username, password)
    listing_file = DEST / ('archive_listing_session.json' if args.session_only else 'archive_listing_day.json')
    cached = json.loads(listing_file.read_text(encoding='utf-8')) if listing_file.exists() and not args.probe else None
    hours = [2, 3] if args.session_only or args.probe else list(range(24))
    if cached:
        hours = []
    urls = cached['urls'] if cached else []
    missing_hours = cached['missing_hours'] if cached else []
    assert all(url.startswith(BASE) for url in urls), 'Cached URLs must belong to the requested CDDIS archive.'
    for hour in hours:
        response = checked_get(session, BASE + f'{hour:02d}/')
        if response is None:
            missing_hours.append(hour)
            continue
        links = Links()
        links.feed(response.text)
        for href in links.hrefs:
            url = urljoin(BASE + f'{hour:02d}/', href)
            name = urlparse(url).path.rsplit('/', 1)[-1]
            if (urlparse(url).hostname == 'cddis.nasa.gov' and
                    re.fullmatch(r'WUH2\w{5}_[RS]_2026266\d{4}_15M_01S_MO\.(crx|rnx)\.gz', name, re.I)):
                if not args.session_only or '20262660245_' in name or '20262660300_' in name:
                    urls.append(url)
        print(f'Listed hour {hour:02d}; matching files so far: {len(urls)}', flush=True)
    if args.probe:
        print('Archive is accessible. Authentication/availability still requires verification.')
        return
    urls = sorted(set(urls))
    if not urls:
        raise FileNotFoundError('Authenticated listing contains no WUH2 1-second files for the requested date/window.')
    listing_file.write_text(json.dumps({'urls':urls,'missing_hours':missing_hours},indent=2),encoding='utf-8')
    (DEST / 'compressed').mkdir(exist_ok=True)
    (DEST / 'rinex').mkdir(exist_ok=True)
    converter = find_converter()
    manifest = {'source':'CDDIS', 'date':str(DATE), 'day_of_year':DOY,
                'session_only':args.session_only, 'missing_hours':missing_hours,
                'listed_files':len(urls), 'files':[]}
    for index, url in enumerate(urls, 1):
        name = urlparse(url).path.rsplit('/', 1)[-1]
        compressed = DEST / 'compressed' / name
        if not compressed.exists():
            response = checked_get(session, url)
            if response is None:
                raise FileNotFoundError(f'Listed file could not be downloaded: {name}')
            data = response.content
            if data[:2] != b'\x1f\x8b':
                raise ValueError('Server returned a non-gzip response; not saved as observation data.')
            gzip.decompress(data)  # Validate the compressed stream before saving.
            compressed.write_bytes(data)
        unpacked = DEST / 'rinex' / name[:-3]
        if not unpacked.exists():
            with gzip.open(compressed, 'rb') as source, unpacked.open('wb') as target:
                shutil.copyfileobj(source, target)
        record = {'url':url, 'compressed_file':str(compressed.relative_to(ROOT)),
                  'sha256':hashlib.sha256(compressed.read_bytes()).hexdigest(),
                  'bytes':compressed.stat().st_size}
        if unpacked.suffix == '.crx' and converter:
            rinex = unpacked.with_suffix('.rnx')
            if not rinex.exists():
                converted = subprocess.run([converter,str(unpacked)], capture_output=True)
                if converted.returncode != 0 or not rinex.exists():
                    raise RuntimeError('Hatanaka conversion failed: ' + converted.stderr.decode(errors='replace'))
            record['rinex'] = inspect_rinex(rinex)
        elif unpacked.suffix == '.rnx':
            record['rinex'] = inspect_rinex(unpacked)
        else:
            record['conversion_status'] = 'CRX2RNX unavailable; original compressed data preserved'
        manifest['files'].append(record)
        (DEST/'download_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        print(f'Downloaded and checked {index}/{len(urls)}: {name}',flush=True)
    print(f'Complete: {len(urls)} files. Expected nominal blocks: {2 if args.session_only else 96}.')
    print('Check the manifest for missing blocks, actual time coverage, and gaps before RTK processing.')
    (DEST/'下载状态.txt').write_text(
        f'已下载 {len(urls)} 个真实观测文件。\n'
        f'范围：{"本次采集两段" if args.session_only else "当天所有可列出的WUH2高频文件"}。\n'
        '完整性、采样间隔、实际历元和缺口见 download_manifest.json。\n', encoding='utf-8')


if __name__ == '__main__':
    try:
        main()
    except (PermissionError, FileNotFoundError, ValueError, requests.RequestException) as error:
        print(f'Not completed: {error}', flush=True)
        raise SystemExit(2)
