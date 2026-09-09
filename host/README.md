# MPU6050/F9P 组合导航 Qt 上位机

[项目主页](../README.md) | 中文 | [English](README_EN.md)

本目录是 GNSS/IMU 实时上位机。程序显示 IMU、GNSS导航状态和天空图，并把带 GPS 时间戳的 IMU、GNSS导航结果与逐星逐频原始观测分别保存成 CSV。具体信号与频点不在界面显示，但仍完整写入RAWX文件，供后续紧组合使用。

## 安装依赖

在仓库根目录执行：

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r host\requirements.txt

也可以使用其他 Python 3.10+ 解释器。在线高德地图使用 PyQtWebEngine；没有高德 Key 时仍可使用内置 WGS-84 本地米制轨迹图。

## 启动

在仓库根目录执行：

    D:\anaconda\envs\allan-toolkit\python.exe host\imu_serial_qt.py

或双击 host/run_imu_serial_qt.bat。

## 使用步骤

1. 确认 STM32 PA9→USB-TTL RX、USB-TTL TX→PA10，GND 共地；下发使用 3.3V TTL。
2. 将 USB-TTL 插入电脑，关闭可能占用串口的串口助手。
3. 点击“刷新串口”，选择对应 COM 端口。
4. 波特率选择 460800，然后点击“连接”。
5. “导航”页显示轨迹、速度、卫星数、PDOP 和天空图；“IMU”页显示传感器曲线。
6. 按需勾选 `IMU`、`GNSS`、`RAWX`、`LOG`，或点击“全选”（包含 LOG）；连接时选择父目录，程序建立形如 `20260908180500` 的独立采集文件夹，并只创建已勾选的文件。LOG 默认不选，勾选后创建 `event.log`。全部不选时仅实时显示，连接期间保存选项锁定。
7. 采集结束先点击“断开”，再拔出 USB-TTL。

高德地图暂不需要 Key 也能采集。获得 Web API Key（以及控制台要求的 securityJsCode）后填入导航页顶部并点击“加载高德地图”。F9P 的 WGS-84 坐标会在显示时转换为 GCJ-02；保存文件始终保留原始 WGS-84 坐标。

“暂停绘图”只停止界面刷新，不停止串口接收和已启用的文件保存。“清空曲线”清除当前显示缓存，不删除已经保存的 CSV。

每次连接会开始一条新的本地轨迹，避免混入上一次连接的位置。本地轨迹使用独立小点绘制，规避部分Windows/PyQtGraph环境下连续路径产生的横线残影。地图不做位置跳点过滤，全部有效定位点和会话中的 `gnss.csv` 保持一致。

本地轨迹的东西向和北向固定为相同米制比例。自动适应状态下，新轨迹接近图框边缘时会缩小视图并保证全部历史轨迹点仍在图框内；使用鼠标滚轮缩放或左键拖动后自动进入“局部查看”，后续新点不再改变人工选择的范围。点击“最佳窗口”可重新显示全部轨迹并恢复自动适应。

## 输入与输出

### 运行日志

网络线程仍在后台维护周期诊断计数，但“日志”页和 `event.log` 不再输出每 10 秒的完整快照。正常情况下只记录启动、数据目录、串口/NTRIP 连接与恢复、GNSS 定位状态变化等关键事件；连续 5 秒无有效 RTCM、MSM 不完整组、重连、串口/文件错误、丢帧和定位降级等异常以红色粗体显示。异常现场压缩为原因、串口已写/未确认、采集丢帧/无效行及 STM32/F9P 错误计数，不输出大段内部缓存状态，也不会输出原始网络正文、请求头或认证信息。

顶部状态栏继续显示累计网络帧、CRC、重连和队列信息，鼠标悬停可查看 MSM 兼容统计。后台 `0.5s` 读等待属于正常轮询，不作为日志事件。日志中的异常快照来自不同线程的近似状态，其中时间间隔不是接收机差分龄期。

“日志”页保留带本机毫秒时间的基站连接/重连原因、收到有效 RTCM 后的恢复提示、串口连接/异常、数据文件创建、GNSS 定位状态变化和合并统计的丢帧/无效行警告。RTK 固定降为浮点/3D、差分丢失或定位解失效会标红；首次获得定位和精度升级属于普通关键信息。恢复提示中的时间从首次报错起算，不是完整断流时长，也不是差分龄期；日志不记录账号密码或逐条观测数据。

支持自动滚动、复制、手动导出 UTF-8 `.log` 和清空。界面仅保留本次进程最近 5000 条，关闭程序后不保留。连接前勾选顶部 `LOG`，本次采集期间的新日志会持续写入同一时间戳目录的 `event.log`，不限 5000 条；不混入前一次采集或连接前的界面历史。低频日志每条刷新到系统缓冲，断开/退出时关闭文件（不保证突然断电落盘）。日志页显示保存状态；写入失败会显示错误并停止 LOG 写入，不主动停止 CSV 采集。

日志页可选择最低输出等级：`INFO`（默认，关键事件及以上）、`WARN`（仅警告和错误）、`ERROR`（仅错误）或 `DEBUG`（全部事件，并每 10 秒追加一条精简链路状态）。每行明确带等级和分类，例如 `[WARN] [链路]`；WARN/ERROR 标红，ERROR 加粗，DEBUG 灰色。选择立即影响后续界面显示和 `event.log` 写入，不补写切换前已过滤的事件。

LOG 默认不勾选，不选时仍可查看和手动导出界面日志。三个 CSV 格式不变。清空日志只清除界面，不截断 `event.log`，不影响轨迹、采集计数或数据文件；清空曲线也不删除日志。手动导出不可覆盖当前正在写入的日志文件。

### NTRIP → STM32 → F9P

#### 分阶段等待与超时保护

已修复“caster 分段发送 MSM 被误判为串口排队”以及“STM32 确认仍在前进、但单帧排队超过 2 秒便误停整条链路”的问题。每条待发帧保留原始接收时刻和完整组释放时刻：MSM 组包时间、完整帧释放后的排队时间及总驻留均用于诊断；排队超过 2 秒只产生一次告警并继续发送，不再单独判定链路死亡。总驻留不是测量历元差分龄期，也不包含串口写入后的硬件传输时间。

1024 字节在途上限、STM32 状态超时、ACK/转发连续 2 秒无进展、串口连续 2 秒写入 0 字节、CRC 校验及 MSM 容量保护均保留，真实的本地堵塞仍停止下发且不重放可能已写出的字节。MSM 只按 `MMI=0`、同星座新历元、站号变化和 64 KiB 容量判断边界；网络墙钟间隔不会删除观测。停止前由串口线程保存即时确认计数、待发字节、组包/发送/总驻留时间，避免只依赖旧的界面快照。此次修复不修改固件、F9P 配置或 CSV 格式。回归覆盖超过 13 秒的 WUH2 分段组包、持续慢速 ACK、真实串口不写入与无 ACK 场景。

#### WUH2 / HPG 1.13 的 MSM 结束标志兼容

本机实测 WUH2 同一组的 GPS/GLO/GAL/SBAS/QZSS/BDS MSM 均带 `multiple message=1`，最后的 NavIC 1137 才为 0。F9P HPG 1.13 回报 1137 未使用，原先出现“收到/使用差分计数增长但仍 3D”的情况。开启以下兼容处理后，实机 NAV-PVT 回报 `carr_soln=2, diffSoln=1`。

`F9pMsmAdapter` 缓存完整 MSM 组（最多 64 KiB，不按网络到达时间截断），移除 F9P 不支持的 1131–1137，把最后一条保留 MSM 的 multiple-message 位清零并重算 CRC24Q。伪距、相位、锁定时间、信号/卫星掩码等其他位完全保留。非 MSM 及未知 RTCM 消息立即原样转发，既不结束也不丢弃 MSM 组。正常的 `MMI=0` 结束组直接释放；若同一星座的新历元先到，则以这个明确的历元边界修复并释放上一组；只有站号变化、MSM 头畸形或超过容量时才丢弃缓存。跨真正的网络重连会清空旧组，不会拼接两个连接的数据。

这是面向本项目 F9P 的转发兼容层，不是通用透明 RTCM 记录器；未来换用支持 NavIC 的接收机需关闭该适配（`NtripClient(..., f9p_compat=False)`）。网络帧计数包含原始 NavIC，实际转发字节数会较小；状态行悬停提示显示移除/重组次数。适配后的 RTK 状态仍由接收机报告，不由 Qt 推算或强制显示。

连接 COM7/460800 后等待 `RTCM 就绪`，点击“基站设置…”填写服务器、端口、挂载点、账号密码，支持从自己选择的 `.bnc` 文件导入。默认 `ntrip.gnsswhu.cn:2101/WUH200CHN0`，点击“连接基站”开始。密码默认仅在内存中；勾选“记住密码”才以本机明文 Qt 设置保存，不进入仓库。不要上传含账号密码的 BNC 文件。

NTRIP 使用独立线程、直接 TCP、v2 请求，支持 HTTP chunked。帧经过 CRC24Q 校验后进入 64 KiB 字节队列，不再按 16 帧限制；突发可暂存，容量满时网络线程等待串口消费。独立串口线程拥有同一 COM7，批量下发、持续读取，并直接处理 STM32 确认，绘图不参与流控。总在途数据仍不超过 1024 字节。单帧排队超过两秒会告警但继续追赶；只有 STM32 状态/确认连续两秒不前进、串口连续写入 0 字节或串口错误才停止基站下发，采集/CSV 保存继续。网络中断五秒后尝试重连，旧连接缓存不会重放。停止基站不停止采集；断开串口/关闭程序会同时停止网络和串口线程。

界面显示网络重连次数、当前排队字节数/时间。串口到 GUI 的缓存上限为 2 MiB；GUI/记录长时间卡住达到上限时会明确停止并报警，不静默丢记录。CSV 解析保存仍在主线程，因此本程序是有缓冲和超时监测的实时采集工具，不承诺硬实时截止时间。

系统 HTTP 代理会被绕过；VPN 若接管 TUN/全局路由，请单独设置该基站直连。该方案不依赖 BNC，测试时停止其他差分注入。仅适用于无需动态 GGA 的真实基站流（当前 WUH2）；没有实现 VRS 的周期 GGA 上报，也没有 TLS。

状态依次显示网络有效帧、STM32 收/转发字节、丢字节/串口错误、F9P 收/使用帧、CRC错误和距接收间隔。最后一项不是观测历元差分龄期。F9P 是否固定由 NAV-PVT 的 `carr_soln` 判断，结果随原 `gnss.csv` 保存，无新增数据文件。

输入为 STM32 协议v3的 IMU、导航、天空图和RAWX记录；RAWX由历元头、观测行和历元尾组成：

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

每次采集文件夹中可选输出三个简写文件（只创建连接前勾选的类型）：

- `imu.csv`：GPS 周/周内微秒、STM32 捕获时间、IMU 原始值及物理量。
- `gnss.csv`：GPS时间和接收时刻、位置/速度及精度、PDOP、定位类型、RTK状态和卫星数。
- `rawx.csv`：接收时间、星座/卫星/信号、中心频率、伪距、载波相位、多普勒、C/N0和质量位。

若同一秒内重复开始采集，文件夹会自动命名为 `20260908180500_01` 等，已有数据不会被覆盖。

`time_valid=1` 才表示GPS时间有效。GNSS文件中的经纬度为WGS-84十进制度。程序只解析协议v3完整记录。

## 打包 Windows EXE

建议使用 PyInstaller 的 onedir 模式。它启动时直接加载目录内文件，不会像 onefile 一样每次解压大型运行环境。

在仓库根目录新建独立打包环境：

    D:\anaconda\envs\allan-toolkit\python.exe -m venv .venv-package
    .\.venv-package\Scripts\python.exe -m pip install --upgrade pip
    .\.venv-package\Scripts\python.exe -m pip install -r host\requirements.txt pyinstaller

执行打包：

    .\.venv-package\Scripts\python.exe -m PyInstaller --noconfirm --clean host\MPU6050_F9P_Navigation.spec

输出位于：

    dist\MPU6050_F9P_Navigation\MPU6050_F9P_Navigation.exe

发布时必须压缩并分发整个 MPU6050_F9P_Navigation 文件夹，不能只复制 EXE。build/、dist/ 和 .venv-package/ 都可删除并重新构建，默认不提交 Git。

## 常见问题

- 点击连接没有数据：检查 COM 口、460800 波特率、PA9→RX 和共地。本机当前 USB-TTL 为 CH340 COM7。
- 串口打开失败：关闭其他串口软件，重新插拔 USB-TTL 后刷新端口。
- 显示乱码或无效行增加：确认固件输出格式和波特率没有改变。
- 刚连接时可能从一个RAWX历元中间开始；程序会丢弃首个历元头之前的零散观测，不把它们计作无效行。同步完成后，字段数错误、非数值内容和缺少历元头的RAWX观测仍会计入“无效行”。
- 运行 .bat 后闪退：在 PowerShell 直接运行 Python 命令查看错误信息。
- EXE 启动慢：确认使用当前 onedir spec，并从完整输出目录启动。

后续命令行采集和 Allan 分析见 [工具说明](../tools/README.md)。
