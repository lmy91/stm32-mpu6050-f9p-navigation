# WUH2 一键下载：默认最少分段，全天可选

推荐双击 `run_gui.bat` 使用 Qt 界面，或双击 `run_download.bat` 选择文件后自动运行。命令行入口是 `wuh2_download_merge.py`。所有工具已统一放在 `tools/station_observation_downloader/`，详见 [README](README.md)。输入流动站 RINEX 3 观测文件，自动读取 GPS 日期和首尾时间，不需要手工填写日期、小时或年积日。

在脚本目录打开 PowerShell，运行：

```powershell
python .\wuh2_download_merge.py "C:\你的数据目录\rover.obs"
```

使用其他流动站文件：

```powershell
python .\wuh2_download_merge.py "C:\你的数据目录\rover.obs"
```

本次下载结果已经存在，再运行不会询问账号或访问服务器。需要补充数据时，程序才提示输入 Earthdata 用户名和隐藏密码；也支持本机已有的 Earthdata netrc 配置及 `EARTHDATA_USERNAME` / `EARTHDATA_PASSWORD` 环境变量。密码不写入脚本或缓存。

## 最终保留的文件

默认直接保存在流动站观测文件所在文件夹，仅下载覆盖流动站范围的最少十五分钟分段并生成匹配时段文件。全天观测默认不下载，流动站和其他原有文件保留：

1. `WUH200CHN_R_YYYYDDD0000_01D_01S_MO.rnx`：仅勾选“下载全天观测”或添加 `--full-day` 后生成；完整 GPS 日期，00:00:00～23:59:59，86,400 个一秒历元。
2. `WUH2_YYYYMMDD_HHMMSS_HHMMSS_1s.rnx`：覆盖流动站采集范围。

例如当前 `rover.obs` 为 GPS 时间 2026-09-23 02:46:15.805～03:06:25.706。基站历元为整数秒，取首时刻向下、尾时刻向上包围，得到 **02:46:15～03:06:26**，共 **1,212 个历元**。提取时保留原始观测值，不插值。

完整全天文件存在且通过校验时直接复用；如果只是换了同一天的流动站文件，重新从全天文件提取相应时段。原有旧时段输出会在新文件生成并验证后清理。

这份流动站默认只需下载 02:45 和 03:00 两个分段；勾选全天模式才需 96 个。所需结果验证成功后清理下载分段和中间文件。允许处理期间直接关闭窗口，当前任务和子程序会停止，完整有效的已有文件保留，下次运行自动复用；传输未完成的那个分段会重新下载。

小型检查缓存保存在流动站文件夹中的 `.wuh2_pipeline/`，不存观测数据和密码，用于重复运行时快速判断文件是否已完成。中间数据使用专用 `.wuh2_work_WUH200CHN_YYYYMMDD/`，所需输出成功验证后清理。若指定独立输出目录，缓存位于该目录的父目录。

## 其他参数

Qt 主界面可勾选“同时下载当天广播星历”，按同一个流动站日期下载 `BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx`。命令行使用 `--broadcast-ephemeris`。有效的本地星历（含旧 `brdcDDD0.YYp/` 目录）直接复用。该项默认关闭，详见 [广播星历说明](README.md#可选当天广播星历)。

```powershell
# 明确禁止网络访问，用已有全天文件生成时段文件
python .\wuh2_download_merge.py "C:\你的数据目录\rover.obs" --offline

# 指定一个独立输出目录
python .\wuh2_download_merge.py "C:\你的数据目录\rover.obs" --output "C:\结果\WUH2_20260923_1s"
```

流动站首尾必须位于同一个 GPS 日期。时间已是 GPS 时间时不加 8 小时。默认测站为 WUH200CHN。

依赖 Python 的 `requests`。本机已安装 CRX2RNX 和 GFZRNX，脚本会自动找到；移到其他电脑时可用 `--crx2rnx`、`--gfzrnx` 指定路径。完整全天文件已存在时，普通时段提取直接由 Python 完成；遇到接收机/头部事件会交由 GFZRNX 处理。

数据来源：`https://cddis.nasa.gov/archive/gnss/data/highrate/YYYY/DDD/YYd/HH/`，首次下载需要 Earthdata 账号。
