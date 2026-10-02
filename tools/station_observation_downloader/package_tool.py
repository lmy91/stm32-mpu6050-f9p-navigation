"""Create a source-only ZIP in tools/; exclude observations, credentials and caches."""
from pathlib import Path
import os
import zipfile

ROOT = Path(__file__).resolve().parent
FILES = [
    '.gitignore', 'README.md', 'requirements.txt', 'choose_python.bat',
    'run_gui.bat', 'run_download.bat', 'one_click_download.py',
    'station_downloader_qt.py', 'wuh2_download_merge.py', 'package_tool.py',
    'broadcast_ephemeris.py', 'test_broadcast_ephemeris.py',
    'test_wuh2_download_merge.py', 'test_station_downloader_qt.py',
    'WUH2_自动下载合并使用说明.md', 'WUH2_1秒数据获取流程.md',
    'legacy/README.md', 'legacy/download_wuh2_1s.py',
    'legacy/merge_verify_wuh2.py', 'legacy/check_cddis_availability.py',
]


def main():
    archive = ROOT.parent/'station_observation_downloader.zip'
    temporary = archive.with_suffix('.zip.part')
    for name in FILES:
        path = ROOT/name
        if not path.is_file() or path.is_symlink() or not path.resolve().is_relative_to(ROOT):
            raise ValueError(f'Missing or invalid package source: {name}')
    with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED) as output:
        for name in FILES:
            output.write(ROOT/name, f'{ROOT.name}/{name}')
    with zipfile.ZipFile(temporary) as output:
        if output.testzip() is not None or len(output.namelist()) != len(FILES):
            raise ValueError('ZIP integrity check failed')
    os.replace(temporary, archive)
    print(f'{archive}\n{len(FILES)} files, {archive.stat().st_size:,} bytes')


if __name__ == '__main__':
    main()
