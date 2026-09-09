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

本地轨迹的东西向和北向固定为相同米制比例。新轨迹接近图框边缘时，视图会自动缩小并保证全部历史轨迹点仍在图框内；平移或缩放后也可点击“最佳窗口”手动恢复等比例最佳视图。

## 输入与输出

### 运行日志

调试信息默认启用：网络线程每 10 秒汇总，连续 5 秒没有有效 RTCM 时预警，异常重连前保存现场。包含会话编号、连接阶段、TCP/解包正文/有效帧增量、分别距上次到达的时间、HTTP/chunk 缓存、RTCM 残留与重同步丢弃字节、CRC 错误、MSM 待组和下发队列。另附串口线程近似快照：写入/未确认字节、最大发送等待、GUI 接收队列、STM32/F9P 累计计数及当前定位状态。不会输出原始网络正文、请求头或认证信息。

`TCP` 包含 HTTP 头和 chunk framing，`正文` 是 HTTP 解包后的字节；均为本次网络连接计数，界面顶部网络计数则跨自动重连累计。`0.5s读等待超时` 是正常轮询可能产生的计数，不等同于重连次数。TCP 到达间隔大通常说明没有收到网络字节；TCP 持续到达而正文不增长需检查 HTTP/chunk 状态；正文增长但有效帧不增长需检查 CRC、残留和重同步计数。这些信息定位阻塞环节，不单凭它们认定 VPN 或服务器故障。各线程快照不同步，间隔不是接收机差分龄期。

“日志”页保留带本机毫秒时间的基站连接/重连原因、收到有效 RTCM 后的恢复提示、串口连接/异常、数据文件创建、GNSS 定位状态变化和合并统计的丢帧/无效行警告。恢复提示中的时间从首次报错起算，不是完整断流时长，也不是差分龄期；日志不记录账号密码或逐条观测数据。

支持自动滚动、复制、手动导出 UTF-8 `.log` 和清空。界面仅保留本次进程最近 5000 条，关闭程序后不保留。连接前勾选顶部 `LOG`，本次采集期间的新日志会持续写入同一时间戳目录的 `event.log`，不限 5000 条；不混入前一次采集或连接前的界面历史。低频日志每条刷新到系统缓冲，断开/退出时关闭文件（不保证突然断电落盘）。日志页显示保存状态；写入失败会显示错误并停止 LOG 写入，不主动停止 CSV 采集。

LOG 默认不勾选，不选时仍可查看和手动导出界面日志。三个 CSV 格式不变。清空日志只清除界面，不截断 `event.log`，不影响轨迹、采集计数或数据文件；清空曲线也不删除日志。手动导出不可覆盖当前正在写入的日志文件。

### NTRIP → STM32 → F9P

#### 分阶段等待与超时保护

已修复“MSM 组包等待被重复算作串口排队”的问题。每条待发帧保留原始接收时刻和完整组释放时刻：组包最多 2 秒；从组释放到串口写入的等待（含网络待发队列、流控和部分写入）最多 2 秒；从接收到写入的主机总驻留最多 4 秒。取出队列和部分写入不会刷新起始时间。该总驻留不是测量历元差分龄期，也不包含串口写入后的硬件传输时间。

1024 字节在途上限、STM32 ACK/转发无进展超时、CRC 校验、缺失 MSM 结束标志保护均保留，真实堵塞仍停止下发且不重放可能已写出的字节。停止前由串口线程保存即时确认计数、待发字节、组包/发送/总驻留时间，避免只依赖旧的界面快照。此次修复不修改固件、F9P 配置或 CSV 格式。模拟回归覆盖 1.8 秒组包 + 0.3 秒跨流控窗口分段发送：不再误停；真实发送超时与无 ACK 仍会停止。

#### WUH2 / HPG 1.13 的 MSM 结束标志兼容

本机实测 WUH2 同一组的 GPS/GLO/GAL/SBAS/QZSS/BDS MSM 均带 `multiple message=1`，最后的 NavIC 1137 才为 0。F9P HPG 1.13 回报 1137 未使用，原先出现“收到/使用差分计数增长但仍 3D”的情况。开启以下兼容处理后，实机 NAV-PVT 回报 `carr_soln=2, diffSoln=1`。

`F9pMsmAdapter` 缓存完整 MSM 组（最多 64 KiB/两秒），移除 F9P 不支持的 1131–1137，把最后一条保留 MSM 的 multiple-message 位清零并重算 CRC24Q。伪距、相位、锁定时间、信号/卫星掩码等其他位完全保留。普通已正确结束的组不修改；站号变化、同星座历元变化、缺少结束标志/超限会报错，不伪造完整历元。非 MSM 消息照常转发，跨网络重连不会拼接旧组。

这是面向本项目 F9P 的转发兼容层，不是通用透明 RTCM 记录器；未来换用支持 NavIC 的接收机需关闭该适配（`NtripClient(..., f9p_compat=False)`）。网络帧计数包含原始 NavIC，实际转发字节数会较小；状态行悬停提示显示移除/重组次数。适配后的 RTK 状态仍由接收机报告，不由 Qt 推算或强制显示。

连接 COM7/460800 后等待 `RTCM 就绪`，点击“基站设置…”填写服务器、端口、挂载点、账号密码，支持从自己选择的 `.bnc` 文件导入。默认 `ntrip.gnsswhu.cn:2101/WUH200CHN0`，点击“连接基站”开始。密码默认仅在内存中；勾选“记住密码”才以本机明文 Qt 设置保存，不进入仓库。不要上传含账号密码的 BNC 文件。

NTRIP 使用独立线程、直接 TCP、v2 请求，支持 HTTP chunked。帧经过 CRC24Q 校验后进入 64 KiB 字节队列，不再按 16 帧限制；突发可暂存，容量满时网络线程短暂等待消费，持续积压两秒才报错。独立串口线程拥有同一 COM7，批量下发、持续读取，并直接处理 STM32 确认，绘图不参与流控。总在途数据仍不超过 1024 字节。积压两秒、确认超时或下行串口错误会停止基站下发，采集/CSV 保存继续。网络中断五秒后尝试重连，旧缓存不会重放。停止基站不停止采集；断开串口/关闭程序会同时停止网络和串口线程。

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
