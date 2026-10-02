# 串口采集与接收机观测

[工具索引](../README.md)

| 文件 | 用途 |
| --- | --- |
| `capture_serial.py` | 协议 v3 采集，保存 IMU/GNSS/RAWX/sync；支持 Pi 第二串口 UBX 与实时状态 |
| `decode_rawx.py` | RAWX 完整性与信号质量检查 |
| `inspect_f9p.py` | F9P 原生 USB 口只读 UBX 查询 |
| `test_capture_serial.py`、`test_decode_rawx.py` | 采集与解码回归测试 |

在仓库根目录运行：

```powershell
python -m pip install -r tools/requirements.txt
python tools/acquisition/capture_serial.py COM7 --hours 12
python tools/acquisition/decode_rawx.py data/decoded/<session>/rawx.csv
python tools/acquisition/inspect_f9p.py COM3 --details
python -X utf8 -m unittest discover -s tools/acquisition -p 'test_*.py'
```

采集默认输出仍为 `data/decoded/<session>/`，端口须按实际设备替换；同一串口只能由一个程序使用。完整协议、sync 字段及采集选项见工具索引。查询 F9P 使用原生 USB 口，不能把示例 COM3 直接替换为 STM32 的串口。
