"""Double-click launcher: select a rover file, then run the full pipeline."""
from pathlib import Path
import sys

import requests
from wuh2_download_merge import main, say


def launch():
    arguments = sys.argv[1:]
    if not arguments:
        from PyQt5.QtWidgets import QApplication, QFileDialog
        app = QApplication(sys.argv)
        path, _ = QFileDialog.getOpenFileName(None, '选择流动站 RINEX 3 观测文件',
                                             str(Path.home()),
                                             '观测文件 (*.obs *.rnx *.*o);;所有文件 (*)')
        if not path:
            return 0
        arguments = [path]
    try:
        main(arguments)
    except (OSError, ValueError, RuntimeError, requests.RequestException, KeyError, IndexError) as error:
        say(f'未完成：{error}')
        return 1
    return 0


if __name__ == '__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    raise SystemExit(launch())
