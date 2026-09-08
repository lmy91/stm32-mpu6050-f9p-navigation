# MPU6050/F9P 组合导航 Qt 上位机

[项目主页](../README.md) | 中文 | [English](README_EN.md)

本目录是 GNSS/IMU 实时上位机。程序显示三轴加速度、三轴角速度、温度、GNSS 位置轨迹、NED/地面速度、定位状态、卫星数、PDOP 和卫星天空图，并把带 GPS 时间戳的 IMU 与 GNSS 导航结果分别保存成 CSV。

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
6. 勾选“分别保存 IMU/GNSS CSV”，连接时选择目录，程序会自动创建两个带时间的文件。
7. 采集结束先点击“断开”，再拔出 USB-TTL。

高德地图暂不需要 Key 也能采集。获得 Web API Key（以及控制台要求的 securityJsCode）后填入导航页顶部并点击“加载高德地图”。F9P 的 WGS-84 坐标会在显示时转换为 GCJ-02；保存文件始终保留原始 WGS-84 坐标。

“暂停绘图”只停止界面刷新，不停止串口接收和已启用的文件保存。“清空曲线”清除当前显示缓存，不删除已经保存的 CSV。

## 输入与输出

输入为 STM32 的三种带类型记录：

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used

输出为两个文件：

- `imu_gnss_time_时间.csv`：GPS 周/周内微秒、STM32 捕获时间、IMU 原始值及物理量。
- `gnss_nav_时间.csv`：GPS时间和接收时刻、位置/速度及精度、PDOP、定位类型、RTK状态和卫星数。

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
- 运行 .bat 后闪退：在 PowerShell 直接运行 Python 命令查看错误信息。
- EXE 启动慢：确认使用当前 onedir spec，并从完整输出目录启动。

后续命令行采集、解码和 Allan 分析见 [工具说明](../tools/README.md)。
