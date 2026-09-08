# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path
import sys

HOST_DIR = Path(SPECPATH)
CONDA_BIN = Path(sys.base_prefix) / 'Library' / 'bin'
CONDA_RUNTIME_DLLS = (
    'ffi.dll',
    'libexpat.dll',
    'libssl-3-x64.dll',
    'libcrypto-3-x64.dll',
    'liblzma.dll',
    'libbz2.dll',
)
EXTRA_BINARIES = [
    (str(CONDA_BIN / name), '.')
    for name in CONDA_RUNTIME_DLLS
    if (CONDA_BIN / name).is_file()
]

a = Analysis(
    [str(HOST_DIR / 'imu_serial_qt.py')],
    pathex=[],
    binaries=EXTRA_BINARIES,
    datas=[],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='MPU6050_F9P_Navigation',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='MPU6050_F9P_Navigation',
)
