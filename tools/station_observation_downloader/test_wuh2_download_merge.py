"""Offline regression checks; use temporary data, never alter downloaded observations."""
from contextlib import redirect_stdout
from datetime import datetime,timedelta
import gzip
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import requests
import wuh2_download_merge as pipeline

STATION = 'WUH200CHN'
DAY = datetime(2026,9,23)


def header(first,last):
    return ('     3.05           OBSERVATION DATA    M (MIXED)'.ljust(60)+'RINEX VERSION / TYPE\n'+
            STATION.ljust(60)+'MARKER NAME\n'+
            '     1.000'.ljust(60)+'INTERVAL\n'+
            'G    1 C1C'.ljust(60)+'SYS / # / OBS TYPES\n'+
            pipeline.header_time(first,'TIME OF FIRST OBS')+
            pipeline.header_time(last,'TIME OF LAST OBS')+' '.ljust(60)+'END OF HEADER\n')


def observations(count=3,start=DAY):
    chunks = [header(start,start+timedelta(seconds=count-1))]
    for i in range(count):
        t = start+timedelta(seconds=i)
        chunks.append(f'> {t:%Y %m %d %H %M} {t.second:10.7f}  0  1\nG01  22000000.000  \n')
    return ''.join(chunks).encode('ascii')


class FakeDownload:
    def __init__(self,payload):
        self.payload = payload
        self.calls = 0

    def download(self,url,target):
        self.calls += 1
        target.write_bytes(self.payload)


class PipelineTests(unittest.TestCase):
    def test_minimal_slots_includes_rounded_boundary_and_full_day(self):
        self.assertEqual(pipeline.required_slots(DAY+timedelta(hours=2,minutes=46),DAY+timedelta(hours=3,minutes=6)),['0245','0300'])
        self.assertEqual(pipeline.required_slots(DAY,DAY+timedelta(minutes=15)),['0000','0015'])
        self.assertEqual(pipeline.required_slots(DAY,DAY+timedelta(minutes=14,seconds=59)),['0000'])
        self.assertEqual(len(pipeline.required_slots(DAY,DAY,True)),96)

    def test_minimal_download_actual_native_merge_repeat_and_full_day_selection(self):
        merger = pipeline.tool_path(None,'GFZRNX','gfzrnx',[r'D:\Dr\algorithm\FAST-main\fast\bin\gfzrnx_win.exe'])
        if not merger:
            self.skipTest('Native integration check needs GFZRNX')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rover = root/'rover.obs'
            rover.write_text(header(DAY+timedelta(hours=2,minutes=46,seconds=15.805),DAY+timedelta(hours=3,minutes=6,seconds=25.706)),encoding='ascii')
            listings,downloads = [],[]
            def listing(client,url):
                hour = int(url.rstrip('/').rsplit('/',1)[-1])
                listings.append(hour)
                return ''.join(f'<a href="{STATION}_R_2026266{hour:02d}{minute:02d}_15M_01S_MO.rnx.gz">file</a>' for minute in (0,15,30,45))
            def download(client,url,target):
                timestamp = target.name.split('_')[2]
                start = datetime.strptime(timestamp,'%Y%j%H%M')
                downloads.append(timestamp[-4:])
                target.write_bytes(gzip.compress(observations(900,start)))
                client.downloads += 1
            with patch.object(pipeline.Client,'listing',listing),patch.object(pipeline.Client,'download',download),redirect_stdout(io.StringIO()):
                first = pipeline.main([str(rover),'--gfzrnx',merger])
                self.assertEqual(listings,[2,3])
                self.assertEqual(downloads,['0245','0300'])
                self.assertEqual(first['required_segments'],2)
                self.assertEqual(first['session']['epochs'],1212)
                self.assertIsNone(first['daily'])
                self.assertEqual(len(list(root.glob('WUH*.rnx'))),1)
                second = pipeline.main([str(rover),'--offline'])
                self.assertEqual(second['downloaded_this_run'],0)
                self.assertEqual(len(downloads),2)
                listings.clear()
                downloads.clear()
                full = pipeline.main([str(rover),'--full-day','--gfzrnx',merger])
                self.assertEqual(full['required_segments'],96)
                self.assertEqual(len(downloads),96)
                self.assertEqual(full['daily']['epochs'],86400)
                self.assertEqual(len(list(root.glob('WUH*.rnx'))),2)
                self.assertTrue(full['full_day_requested'])

    def test_resume_from_valid_uncompressed_segment_without_download(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rover = root/'rover.obs'
            rover.write_text(header(DAY+timedelta(seconds=10.3),DAY+timedelta(seconds=20.7)),encoding='ascii')
            work = root/'.wuh2_work_WUH200CHN_20260923'/'rinex'
            work.mkdir(parents=True)
            (work/'WUH200CHN_R_20262660000_15M_01S_MO.rnx').write_bytes(observations(900))
            with patch.object(pipeline.Client,'authenticate',side_effect=AssertionError('No network')),redirect_stdout(io.StringIO()):
                result = pipeline.main([str(rover),'--offline'])
            self.assertEqual(result['session']['epochs'],12)
            self.assertEqual(result['downloaded_this_run'],0)
            self.assertEqual(len(list(root.glob('WUH*.rnx'))),1)

    def test_output_lock_prevents_parallel_write_and_releases(self):
        with tempfile.TemporaryDirectory() as folder:
            lock = Path(folder)/'output.lock'
            with pipeline.output_lock(lock):
                with self.assertRaises(RuntimeError):
                    with pipeline.output_lock(lock):
                        self.fail('Parallel write was allowed')
            with pipeline.output_lock(lock):
                pass

    def test_noninteractive_missing_credentials_does_not_prompt(self):
        client = pipeline.Client(non_interactive=True)
        with patch.dict(os.environ, {'EARTHDATA_USERNAME':'', 'EARTHDATA_PASSWORD':''}), patch.object(pipeline.netrc, 'netrc', side_effect=OSError), \
             patch('builtins.input', side_effect=AssertionError('No console prompt allowed')), \
             patch.object(pipeline.getpass, 'getpass', side_effect=AssertionError('No password prompt allowed')):
            with self.assertRaises(PermissionError):
                client.authenticate()

    def test_existing_day_no_network_two_outputs_and_new_rover_range(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dest = root/'WUH2_20260923_1s'
            dest.mkdir()
            daily = dest/'WUH200CHN_R_20262660000_01D_01S_MO.rnx'
            daily.write_bytes(observations(86400))
            work = dest/'.wuh2_work_WUH200CHN_20260923'
            (work/'compressed').mkdir(parents=True)
            (work/'compressed'/'intermediate.gz').write_bytes(b'old')
            (work/'rinex').mkdir()
            (work/'rinex'/'intermediate.rnx').write_bytes(b'old')
            rover = root/'rover.obs'
            rover.write_text(header(DAY+timedelta(seconds=10.3),DAY+timedelta(seconds=20.7)),encoding='ascii')
            with patch.object(pipeline.Client,'authenticate',side_effect=AssertionError('Network forbidden')),redirect_stdout(io.StringIO()):
                first = pipeline.main([str(rover),'--output',str(dest),'--offline'])
                mtimes = {p.name:p.stat().st_mtime_ns for p in dest.iterdir()}
                second = pipeline.main([str(rover),'--output',str(dest),'--offline'])
                self.assertEqual(mtimes,{p.name:p.stat().st_mtime_ns for p in dest.iterdir()})
                rover.write_text(header(DAY+timedelta(seconds=30.3),DAY+timedelta(seconds=40.7)),encoding='ascii')
                third = pipeline.main([str(rover),'--output',str(dest),'--offline'])
            self.assertEqual(first['downloaded_this_run'],0)
            self.assertEqual(second['session']['epochs'],12)
            self.assertEqual(third['session']['first_gps'],'2026-09-23T00:00:30')
            self.assertEqual(len(list(dest.iterdir())),2)
            self.assertTrue(all(p.suffix=='.rnx' for p in dest.iterdir()))
            self.assertTrue(daily.exists())

    def test_good_gzip_skipped_bad_gzip_repaired(self):
        with tempfile.TemporaryDirectory() as folder:
            dest = Path(folder)
            (dest/'compressed').mkdir()
            (dest/'rinex').mkdir()
            name = 'WUH200CHN_R_20262660000_15M_01S_MO.rnx.gz'
            url = 'https://cddis.nasa.gov/archive/gnss/data/highrate/2026/266/26d/00/'+name
            payload = gzip.compress(observations())
            compressed = dest/'compressed'/name
            compressed.write_bytes(payload)
            client = FakeDownload(payload)
            state = {'parts':{},'merges':{}}
            pipeline.ensure_rinex(url,dest,STATION,client,state,{},None)
            pipeline.ensure_rinex(url,dest,STATION,client,state,{},None)
            self.assertEqual(client.calls,0)
            compressed.write_bytes(payload[:10])
            with redirect_stdout(io.StringIO()):
                pipeline.ensure_rinex(url,dest,STATION,client,state,{},None)
            self.assertEqual(client.calls,1)
            pipeline.check_gzip(compressed)
            self.assertEqual(len(list((dest/'compressed').glob('*.invalid.*'))),1)

    def test_interrupted_download_retries_without_finalizing_partial(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'data.gz'
            payload = gzip.compress(observations())
            client = pipeline.Client()
            calls = []
            class Response:
                def __enter__(self): return self
                def __exit__(self,*args): pass
                def iter_content(self,size):
                    yield payload[:10]
                    if len(calls)==1:
                        raise requests.exceptions.ChunkedEncodingError('interrupted')
                    yield payload[10:]
            def response(url,stream=False):
                self.assertFalse(target.exists())
                calls.append(url)
                return Response()
            with patch.object(client,'response',side_effect=response),patch.object(pipeline.time,'sleep'),redirect_stdout(io.StringIO()):
                client.download('https://cddis.nasa.gov/example',target)
            self.assertEqual(len(calls),2)
            self.assertEqual(target.read_bytes(),payload)
            self.assertFalse(target.with_name('data.gz.part').exists())

    def test_failed_subset_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            daily = root/'day.rnx'
            daily.write_bytes(observations())
            output = root/'subset.rnx'
            output.write_bytes(b'preserve me')
            with redirect_stdout(io.StringIO()),self.assertRaises(ValueError):
                pipeline.make_subset(daily,output,DAY,DAY+timedelta(seconds=4),STATION,{},None)
            self.assertEqual(output.read_bytes(),b'preserve me')

    def test_receiver_event_requires_native_crop(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root/'event.rnx'
            source.write_text(header(DAY,DAY+timedelta(seconds=2))+
                              '> 2026 09 23 00 00  0.0000000  4  1\n'+
                              'New receiver state'.ljust(60)+'COMMENT\n',encoding='ascii')
            with self.assertRaises(pipeline.NativeCropRequired):
                pipeline.crop_stream(source,root/'subset.rnx',DAY+timedelta(seconds=1),DAY+timedelta(seconds=2))

    def test_cleanup_rejects_link_outside_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dest = root/'out'
            outside = root/'outside'
            dest.mkdir()
            outside.mkdir()
            folder_path = dest/'.wuh2_work_WUH200CHN_20260923'
            folder_path.mkdir()
            original_resolve = Path.resolve
            def resolve(path,*args,**kwargs):
                return outside if path==folder_path else original_resolve(path,*args,**kwargs)
            with patch.object(Path,'resolve',resolve),self.assertRaises(ValueError):
                pipeline.cleanup_intermediate(dest,[],STATION,DAY.date())
            self.assertTrue(outside.exists())
            self.assertTrue(folder_path.exists())

    def test_rover_without_last_header_reads_actual_epochs(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'rover.obs'
            content = observations().decode('ascii')
            content = ''.join(line for line in content.splitlines(keepends=True)
                              if line[60:].strip()!='TIME OF LAST OBS')
            path.write_text(content,encoding='ascii')
            self.assertEqual(pipeline.rover_times(path),(DAY,DAY+timedelta(seconds=2)))


if __name__=='__main__':
    unittest.main()
