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

1. 确认 STM32 PA9→USB-TTL RX，二者 GND 共地。
2. 将 USB-TTL 插入电脑，关闭可能占用串口的串口助手。
3. 点击“刷新串口”，选择对应 COM 端口。
4. 波特率选择 460800，然后点击“连接”。
5. “导航”页显示轨迹、速度、卫星数、PDOP 和天空图；“IMU”页显示传感器曲线。
6. 按需勾选 `IMU`、`GNSS`、`RAWX`，或点击“全选”；连接时选择父目录，程序建立形如 `20260908180500` 的独立采集文件夹，并只创建已勾选的数据文件。全部不选时仅实时显示。
7. 采集结束先点击“断开”，再拔出 USB-TTL。

高德地图暂不需要 Key 也能采集。获得 Web API Key（以及控制台要求的 securityJsCode）后填入导航页顶部并点击“加载高德地图”。F9P 的 WGS-84 坐标会在显示时转换为 GCJ-02；保存文件始终保留原始 WGS-84 坐标。

“暂停绘图”只停止界面刷新，不停止串口接收和已启用的文件保存。“清空曲线”清除当前显示缓存，不删除已经保存的 CSV。

每次连接会开始一条新的本地轨迹，避免混入上一次连接的位置。本地轨迹使用独立小点绘制，规避部分Windows/PyQtGraph环境下连续路径产生的横线残影。地图不做位置跳点过滤，全部有效定位点和会话中的 `gnss.csv` 保持一致。

本地轨迹的东西向和北向固定为相同米制比例。新轨迹接近图框边缘时，视图会自动缩小并保证全部历史轨迹点仍在图框内；平移或缩放后也可点击“最佳窗口”手动恢复等比例最佳视图。

## 输入与输出

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

后续命令行采集、解码和 Allan 分析见 [工具说明](../tools/README.md)。
