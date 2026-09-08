# MPU6050/F9P 数据采集与 Allan 分析工具

[项目主页](../README.md) | 中文 | [English](README_EN.md)

工具与当前STM32串口协议v3配套。默认从PA9/USART1以460800 bit/s接收数据，并分别保存带GPS时间戳的IMU、1 Hz GNSS导航结果和1 Hz RAWX逐星逐频原始观测。

## 工具

| 文件 | 用途 | 默认输出 |
| --- | --- | --- |
| `capture_serial.py` | 无界面采集 | 会话文件夹中的 IMU、GNSS导航、RAWX CSV |
| `allan_noise_identification.py` | 直接读取标准 IMU CSV，辨识 Allan 随机误差 | `data/allan_results/` |

## 安装

在仓库根目录运行：

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt

也可使用其他 Python 3.10+ 解释器。Qt 上位机、命令行采集器和串口助手不能同时打开同一个 COM 口。

## 1. 命令行采集

当前电脑实测 USB-TTL 为 COM7：

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 12

默认波特率为 460800。`--hours 0` 表示持续采集，按 Ctrl+C 会安全关闭文件。与Qt一致，每次采集建立独立会话文件夹，默认生成：

    data\decoded\YYYYMMDDHHMMSS\imu.csv
    data\decoded\YYYYMMDDHHMMSS\gnss.csv
    data\decoded\YYYYMMDDHHMMSS\rawx.csv

只保存指定类型：

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --save imu gnss

`--save imu`、`--save gnss`、`--save rawx` 可任意组合；默认三项全选。同一秒重复启动时会增加 `_01` 后缀，已有数据不会被覆盖。

采集器统计IMU丢帧、无效行和卫星记录；`SAT`/`SAT_END` 用于计数，不重复写入GNSS导航表。

## 2. Allan 随机误差辨识

采集器或 Qt 上位机在采集时已经生成标准物理量IMU CSV，可直接输入：

    D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\20260908180500\imu.csv --rate 100 --skip-minutes 30 --points 90

`--rate` 是名义采样率，当前为 100 Hz；`--skip-minutes` 用于跳过预热；`--points` 必须至少为 30。结果包括 Allan 曲线、稳定性曲线、参数 CSV 及中文判读报告。

## 当前串口协议

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

标准 IMU CSV：

    sample,gps_week,gps_tow_us,time_valid,timer_us,time_s,dt_s,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h

标准 GNSS CSV：

    gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,gnss_fix_ok,diff_soln,lat_deg,lon_deg,hmsl_m,h_acc_m,v_acc_m,vel_n_m_s,vel_e_m_s,vel_d_m_s,ground_speed_m_s,s_acc_m_s,pdop

`time_valid=1` 表示GPS时间有效；经纬度为WGS-84。采集器只接受协议v3完整记录。角速度以deg/h保存，Allan工具内部转换为rad/s。

RAWX CSV 将位模式还原为接收机原始浮点值，并给出 `signal` 和 `frequency_mhz`。GLONASS中心频率会结合 `freq_id` 的频率槽计算；未知的新信号仍保留原始ID，不会丢弃观测。

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
