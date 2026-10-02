"""Inspect CDDIS directory availability using credentials only from process env."""
import concurrent.futures
import json
import os
from pathlib import Path
import re
from urllib.parse import urlparse
from download_wuh2_1s import EarthdataSession

PATHS = [
    '/archive/gnss/data/highrate/2026/266/',
    '/archive/gnss/data/highrate/2026/266/26d/02/',
    '/archive/gnss/data/highrate/2026/266/26d/03/',
]

def probe(path):
    with EarthdataSession() as session:
        session.auth = (os.environ['EARTHDATA_USERNAME'], os.environ['EARTHDATA_PASSWORD'])
        response = session.get('https://cddis.nasa.gov'+path, timeout=(15,30))
        text = response.text if 'text' in response.headers.get('Content-Type','') else ''
        titles = re.findall(r'<title[^>]*>(.*?)</title>', text, re.I|re.S)
        hrefs = re.findall(r'href=[\"\']([^\"\']+)', text, re.I)
        selected = [href.strip() for href in hrefs if 'WUH2' in href.upper()]
        record = {'path':path,'status':response.status_code,
                  'final_host':urlparse(response.url).hostname,
                  'title':titles[:1], 'response_date':response.headers.get('Date'),
                  'selected_links':selected[:100], 'link_examples':hrefs[:15]}
        print(json.dumps(record, ensure_ascii=False), flush=True)
        return record

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    records = list(pool.map(probe, PATHS))
target = Path(__file__).resolve().parent/'WUH2_20260923_1s'/'cddis_availability.json'
target.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
