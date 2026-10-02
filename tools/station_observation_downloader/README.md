# WUH2 一键下载与合并工具

选择流动站 RINEX 3 观测文件，自动识别 GPS 日期和首尾时间。默认只下载覆盖该时段所需的最少十五分钟分段，解压转换、合并并提取匹配时段；全天观测和当天广播星历分别为可选项。

## 双击使用

1. 双击 **`run_gui.bat`**，打开 Qt 界面。
2. 选择或拖入流动站 `.obs` / `.rnx` / RINEX 3 格式的 `.*o` 文件。界面自动显示 GPS 日期、年积日和时段。
3. 默认只下载匹配时段；需要全天数据时勾选 **下载全天观测**，需要星历时勾选 **同时下载当天广播星历**。需要联网下载基站观测时填写 Earthdata 用户名和密码，点击 **一键下载 / 合并 / 提取**。
4. 完成后点击 **打开结果文件夹**。

**默认保存位置就是流动站观测文件所在文件夹**，不另建日期子文件夹。也可在界面更改保存位置。账户密码不保存到配置、脚本、日志或缓存；仅用于本次下载的进程内存。支持本机已有 netrc 登录配置。

双击 **`run_download.bat`** 可通过文件选择框运行同一套完整流程；也可以把流动站文件拖到这个 BAT 上。此入口在控制台显示日志，首次缺少登录配置时提示输入账号和隐藏密码。

## 观测范围和输出

默认生成一个匹配时段观测文件。程序先将首时刻向下、尾时刻向上取整秒，再选择包含这些整数秒的最少分段；如果尾时刻落在下一个十五分钟分段的首秒，该分段也会包含。只读取所需小时的服务器目录。

勾选 **下载全天观测** 才会下载 96 个分段，额外生成全天文件。默认不勾选；已有全天文件会保留并按需复用。

| 文件 | 内容 |
| --- | --- |
| `WUH200CHN_R_YYYYDDD0000_01D_01S_MO.rnx` | 勾选全天观测后生成；GPS 当天 00:00:00～23:59:59，86,400 个连续一秒历元 |
| `WUH2_YYYYMMDD_HHMMSS_HHMMSS_1s.rnx` | 覆盖流动站实际采集时间的基站观测 |

例如流动站 GPS 时间为 2026-09-23 02:46:15.805～03:06:25.706，默认只需 **02:45 和 03:00 两个分段**，生成匹配范围 **02:46:15～03:06:26**，共 **1,212 个历元**。已有匹配时段文件时直接复用。保留原始观测，不插值，时间使用 GPS，不加 8 小时。

已有匹配时段文件或完整全天文件校验通过后直接复用，不联网、不要求登录。下载或转换中断后，完整有效的压缩分段或已转换分段会复用。输出成功验证后删除本工具的分段工作目录和同日旧时段输出；流动站、已有全天文件和其他原有文件保留。

中间数据存入 `.wuh2_work_WUH200CHN_YYYYMMDD/`；小型状态缓存存入 `.wuh2_pipeline/`（默认位于流动站文件夹，使用独立输出目录时位于其父目录）。缓存只用于校验与复用，不存密码。

**下载和处理过程中可以直接关闭窗口**。关闭会停止当前任务及转换/合并子程序，不在后台继续下载。完整文件保留，半成品不标记为完成；下次选择同一文件和保存位置可继续，已下载并校验通过的分段不重复下载。下载传输中断的那个半成品会重新下载。

## 可选：当天广播星历

主界面提供 **同时下载当天广播星历** 勾选项，默认不勾选。勾选后，使用同一个流动站文件识别的 GPS 日期下载 IGS 全天混合 GNSS 广播星历：

`BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx`

新下载的星历与观测结果保存在同一个输出文件夹。已存在且校验通过的星历直接复用，包括旧目录 `brdcDDD0.YYp/` 中同名文件；复用旧文件时保留其原位置，不额外复制。星历勾选项与全天观测勾选项互相独立。

优先从 BKG 公共归档 `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/` 下载，不需要 Earthdata 登录；该源不可用时尝试 CDDIS 的 `archive/gnss/data/daily/YYYY/DDD/YYp/`，后者需要 Earthdata 登录。程序校验 gzip、RINEX 3 混合导航头部、完整记录及所选日期，校验成功后发布解压文件，压缩中间文件自动清理。

勾选星历但下载失败时，日志会明确显示原因，已经生成的基站观测文件保留；重试时自动复用。离线模式也可以勾选星历，但本地必须已有有效的当天文件。

## 环境和外部工具

当前电脑可直接启动。其他电脑需安装 Python 3.10+，然后：

```powershell
python -m pip install -r requirements.txt
```

启动器优先选择 `STATION_DOWNLOADER_PYTHON` 环境变量指定的 Python，其次包内或仓库的 `.venv`、本机 `D:\Anaconda3\python.exe`、最后 PATH 中的 `python`。

首次下载与合并需要 [CRX2RNX](https://terras.gsi.go.jp/ja/crx2rnx.html) 和 [GFZRNX](https://gnss.gfz.de/services/gfzrnx)。这两个第三方程序没有收入 ZIP：当前电脑已安装并自动识别。其他电脑可以在界面高级选项选择它们的可执行文件，或设置 `CRX2RNX`、`GFZRNX` 环境变量。也支持自行把 `crx2rnx.exe`、`gfzrnx.exe` 放到本工具的 `bin/` 目录。

CDDIS 数据源：`https://cddis.nasa.gov/archive/gnss/data/highrate/YYYY/DDD/YYd/HH/`。首次使用 Earthdata 账号请完成 CDDIS 应用授权。所需分段缺失或请求时段存在缺口时，工具明确报错并保留中间数据；仅在全天模式下要求 96 个完整分段。

## 命令行

从本工具目录执行：

```powershell
python wuh2_download_merge.py "C:\数据\rover.obs"
python wuh2_download_merge.py "C:\数据\rover.obs" --offline
python wuh2_download_merge.py "C:\数据\rover.obs" --output "C:\结果"
python wuh2_download_merge.py "C:\数据\rover.obs" --broadcast-ephemeris
python wuh2_download_merge.py "C:\数据\rover.obs" --full-day
```

也可加 `--crx2rnx "程序路径"`、`--gfzrnx "程序路径"`。流动站必须是已解码的 **RINEX 3 观测**，当前一次处理一个 GPS 日期；跨日或尾时刻需要次日历元的文件请先分割。

## 包内文件

- `station_downloader_qt.py`：Qt 图形界面，后台进程运行与实时日志。
- `wuh2_download_merge.py`：统一下载、校验、合并和提取后端。
- `broadcast_ephemeris.py`：可选当天广播星历下载、校验和复用。
- `one_click_download.py`、`run_download.bat`：文件选择或拖入的一键脚本入口。
- `run_gui.bat`、`choose_python.bat`、`requirements.txt`：启动和依赖。
- `test_*.py`：使用临时合成数据的自动检查，不修改真实观测。
- 两份 `WUH2_*.md`：数据获取流程和补充说明。
- `legacy/`：最初分步下载、合并和目录诊断脚本，仅作历史参考；含原始实验日期，不作为当前入口。

`tools/station_observation_downloader.zip` 是工具源码包，解压后双击 `run_gui.bat`。包内不包含大观测数据、登录信息或运行缓存。
