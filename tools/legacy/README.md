# 历史辅助工具

[工具索引](../README.md)

| 文件 | 用途 |
| --- | --- |
| `bnc_direct_proxy.py` | 旧 BNC 本机直连代理，当前 Qt 直连 NTRIP 不需要它 |
| `deep_dive_timing.py` | 历史 AIM 时序分析 |
| `innovation_dive.py` | 历史新息诊断 |
| `extract_viz_data.py`、`gen_viz_html.py` | 历史数据下采样与 HTML 绘图 |

保留供参考，不属于正式温补或基站下载流程。部分文件使用历史本机绝对路径，运行前须查看并调整顶部输入/输出路径、补齐数据；没有统一默认输出目录。BNC 代理可先用 `python tools/legacy/bnc_direct_proxy.py --help` 查看参数。
