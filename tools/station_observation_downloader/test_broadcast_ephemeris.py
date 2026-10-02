"""Navigation validation, reuse and interrupted download checks; temporary files only."""
from datetime import datetime, timedelta
import gzip
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import requests
import broadcast_ephemeris as navigation
from wuh2_download_merge import Client

DAY = datetime(2026,9,23)


def navigation_bytes(day=DAY):
    header = ('     3.04           N: GNSS NAV DATA    M: MIXED'.ljust(60)+'RINEX VERSION / TYPE\n'+
              ''.ljust(60)+'END OF HEADER\n')
    first = f'G01 {day:%Y %m %d %H %M %S}'+f'{0.0:19.12E}'*3+'\n'
    rest = ('    '+f'{0.0:19.12E}'*4+'\n')*7
    return (header+first+rest).encode('ascii')


class Response:
    def __init__(self, url, payload, interrupted=False):
        self.url = url
        self.payload = payload
        self.interrupted = interrupted

    def __enter__(self): return self
    def __exit__(self, *args): pass
    def raise_for_status(self): pass

    def iter_content(self, size):
        yield self.payload[:10]
        if self.interrupted:
            raise requests.exceptions.ChunkedEncodingError('interrupted')
        yield self.payload[10:]


class BroadcastTests(unittest.TestCase):
    def test_navigation_rejects_wrong_date_truncation_and_html(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'nav.rnx'
            path.write_bytes(navigation_bytes())
            self.assertEqual(navigation.inspect_navigation(path,DAY.date())['records'],1)
            for payload in (navigation_bytes(DAY-timedelta(days=1)), navigation_bytes()[:-100], b'<html>login</html>'):
                path.write_bytes(payload)
                with self.assertRaises(ValueError):
                    navigation.inspect_navigation(path,DAY.date())

    def test_local_and_legacy_reuse_without_authentication_or_network(self):
        for legacy in (False,True):
            with self.subTest(legacy=legacy), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                directory = root/'brdc2660.26p' if legacy else root
                directory.mkdir(exist_ok=True)
                path = directory/navigation.filename(DAY)
                path.write_bytes(navigation_bytes())
                modified = path.stat().st_mtime_ns
                client = Client(offline=True,non_interactive=True)
                with patch.object(client,'authenticate',side_effect=AssertionError('No authentication')), \
                     patch.object(requests,'Session',side_effect=AssertionError('No network')):
                    result = navigation.ensure_broadcast(DAY.date(),root,client,lambda message:None)
                self.assertEqual(result['path'],str(path))
                self.assertEqual(result['downloaded'],0)
                self.assertEqual(modified,path.stat().st_mtime_ns)
                self.assertFalse(any(root.glob('*.gz')))

    def test_public_download_and_repeat_reuse_no_earthdata_credentials(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            client = Client(non_interactive=True)
            def get(session,url,**kwargs):
                self.assertFalse(session.trust_env)
                self.assertIsNone(session.auth)
                self.assertIn('/2026/266/BRDC00IGS_R_20262660000_01D_MN.rnx.gz',url)
                return Response(url,gzip.compress(navigation_bytes()))
            with patch.object(requests.Session,'get',autospec=True,side_effect=get) as download, \
                 patch.object(client,'response',side_effect=AssertionError('CDDIS not needed')):
                first = navigation.ensure_broadcast(DAY.date(),root,client,lambda message:None)
                second = navigation.ensure_broadcast(DAY.date(),root,client,lambda message:None)
            self.assertEqual(download.call_count,1)
            self.assertEqual(first['downloaded'],1)
            self.assertEqual(second['downloaded'],0)
            self.assertEqual(len(list(root.iterdir())),1)
            self.assertEqual(Path(first['path']).read_bytes(),navigation_bytes())
            self.assertEqual(client.downloads,0)  # Observation segment counter stays separate.

    def test_connection_interruption_retries_without_publishing_partial_file(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root/navigation.filename(DAY)
            calls = []
            def get(session,url,**kwargs):
                self.assertFalse(target.exists())
                calls.append(url)
                return Response(url,gzip.compress(navigation_bytes()),interrupted=len(calls)==1)
            with patch.object(requests.Session,'get',autospec=True,side_effect=get), patch.object(navigation.time,'sleep'):
                info = navigation.ensure_broadcast(DAY.date(),root,Client(),lambda message:None)
            self.assertEqual(len(calls),2)
            self.assertEqual(info['downloaded'],1)
            self.assertEqual(len(list(root.iterdir())),1)

    def test_invalid_public_data_uses_authenticated_fallback(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            client = Client(non_interactive=True)
            def public(session,url,**kwargs):
                return Response(url,gzip.compress(navigation_bytes(DAY-timedelta(days=1))))
            def fallback(url,**kwargs):
                self.assertIn('/daily/2026/266/26p/',url)
                return Response(url,gzip.compress(navigation_bytes()))
            with patch.object(requests.Session,'get',autospec=True,side_effect=public), patch.object(client,'response',side_effect=fallback):
                info = navigation.ensure_broadcast(DAY.date(),root,client,lambda message:None)
            self.assertEqual(info['source'],'CDDIS')

    def test_failed_download_preserves_existing_file_and_offline_never_connects(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root/navigation.filename(DAY)
            target.write_bytes(b'original invalid file')
            client = Client(non_interactive=True)
            def invalid(url,**kwargs):
                return Response(url,b'not gzip')
            with patch.object(requests.Session,'get',autospec=True,side_effect=lambda session,url,**kwargs:invalid(url)), \
                 patch.object(client,'response',side_effect=invalid), self.assertRaises(RuntimeError):
                navigation.ensure_broadcast(DAY.date(),root,client,lambda message:None)
            self.assertEqual(target.read_bytes(),b'original invalid file')
            self.assertEqual(len(list(root.iterdir())),1)
            with patch.object(requests,'Session',side_effect=AssertionError('No network')), self.assertRaises(RuntimeError):
                navigation.ensure_broadcast(DAY.date(),root,Client(offline=True),lambda message:None)


if __name__=='__main__':
    unittest.main()
