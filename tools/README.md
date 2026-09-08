# MPU6050/F9P 数据采集、解码与 Allan 分析工具

[项目主页](../README.md) | 中文 | [English](README_EN.md)

工具与当前 STM32 串口协议 v2 配套。默认从 PA9/USART1 的 USB-TTL 串口以 460800 bit/s 接收数据，并将带 GPS 时间戳的 IMU 与 1 Hz GNSS 导航结果分别保存。实验数据统一写入 `data/`，默认不提交 Git。

## 工具

| 文件 | 用途 | 默认输出 |
| --- | --- | --- |
| `capture_serial.py` | 无界面采集当前完整串口流 | `data/decoded/` 中独立的 IMU、GNSS CSV |
| `decode_imu_data.py` | 解码当前/旧版 IMU 文件并画七通道图 | `data/decoded/` |
| `allan_noise_identification.py` | 直接读取标准 IMU CSV，辨识 Allan 随机误差 | `data/allan_results/` |

## 安装

在仓库根目录运行：

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt

也可使用其他 Python 3.10+ 解释器。Qt 上位机、命令行采集器和串口助手不能同时打开同一个 COM 口。

## 1. 命令行采集

当前电脑实测 USB-TTL 为 COM7：

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 12

默认波特率为 460800。`--hours 0` 表示持续采集，按 Ctrl+C 会安全关闭文件。默认生成：

    data\decoded\imu_gnss_time_YYYYMMDD_HHMMSS.csv
    data\decoded\gnss_nav_YYYYMMDD_HHMMSS.csv

同时保留 STM32 的完整原始流：

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 1 --raw-output data\raw\serial_1h.txt

自定义输出路径：

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --imu-output data\decoded\imu.csv --gnss-output data\decoded\gnss.csv

`--output` 是旧版 `--imu-output` 的兼容别名。采集器统计 IMU 丢帧、无效行和卫星记录；`SAT`/`SAT_END` 只写入可选原始流，不重复写入 GNSS 导航表。

## 2. 解码 IMU 文件

解码器接受三类输入：

- 当前带类型的 `IMU,...` 原始串口日志；
- 当前 21 列标准 IMU CSV；
- 旧版 10 列 MPU6050 CSV。

运行：

    D:\anaconda\envs\allan-toolkit\python.exe tools\decode_imu_data.py data\raw\serial_1h.txt

自定义输出：

    D:\anaconda\envs\allan-toolkit\python.exe tools\decode_imu_data.py data\raw\serial_1h.txt --output-csv data\decoded\imu.csv --plot data\decoded\imu.png --rate 100

输出统一为 21 列标准 IMU CSV。脚本流式处理长文件，绘图使用分块均值，避免长时间数据耗尽内存。

## 3. Allan 随机误差辨识

采集器或 Qt 上位机生成的标准 IMU CSV 可直接输入，不需要再次解码：

    D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\imu.csv --rate 100 --skip-minutes 30 --points 90

`--rate` 是名义采样率，当前为 100 Hz；`--skip-minutes` 用于跳过预热；`--points` 必须至少为 30。结果包括 Allan 曲线、稳定性曲线、参数 CSV 及中文判读报告。

## 当前串口协议

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,fix,num_sv,lat_e7,lon_e7,hmsl_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs

标准 IMU CSV：

    sample,gps_week,gps_tow_us,time_valid,timer_us,time_s,dt_s,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h

标准 GNSS CSV：

    gps_week,gps_tow_ms,time_valid,fix,num_sv,lat_deg,lon_deg,hmsl_m,vel_n_m_s,vel_e_m_s,vel_d_m_s,ground_speed_m_s,pdop

`time_valid=1` 表示 GPS 时间有效；经纬度为 WGS-84。角速度以 deg/h 保存，Allan 工具内部转换为 rad/s。

## 长时间采集建议

- 刚上电时等待 F9P 定位并确认 `time_valid=1`。
- Allan 静态测试应刚性固定 IMU，预热约 30 分钟，避免温度突变和线缆扰动。
- 先做 5–10 分钟短测，确认 `lost=0`、`invalid=0`、GNSS 每秒一条，再开始长采集。
- 正在写入的文件应复制快照后分析，不要让两个程序同时写同一文件。

## 常见问题

- 串口占用：关闭 Qt 上位机、u-center 和其他串口程序。
- 乱码或无效行：确认选中 STM32 的 USB-TTL 端口并使用 460800，而不是 C099 自身的 USB 口。
- GNSS 文件没有数据：检查 PA2/PA3 交叉连接、共地及 C099 J4 的 ARD 路由。
- GPS 时间无效：把天线移到能看到天空的位置，等待 F9P 获得有效时间。
- Allan 提示样本太少：减小 `--skip-minutes` 或延长静态采集时间。
