# 数据与链路质检

[工具索引](../README.md)

| 文件 | 用途 |
| --- | --- |
| `analyze_latest_data.py` | 指定会话汇总；不传路径则选择本仓库最新会话 |
| `check_sync.py` | 只读检查同步计数、增量与 backlog |
| `check_rtcm_bridge.py` | 调用 Qt 上位机代码检查 RTCM 转发；需实际硬件 |

```powershell
python -X utf8 tools/quality_control/analyze_latest_data.py data/decoded/<session>
python tools/quality_control/check_sync.py data/decoded/<session>/sync.csv
python tools/quality_control/check_rtcm_bridge.py --help
```

运行位置为仓库根目录。会话检查只读数据；RTCM 检查前关闭占用同一串口的 Qt/BNC 等程序。Qt 链路检查另需安装 [host 依赖](../../host/requirements.txt)。详细参数与计数解释见工具索引和 [RTCM 检查说明](../../docs/RTCM_RTK_validation.md)。
