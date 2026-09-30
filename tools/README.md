# MPU6050/F9P 数据采集与 Allan 分析工具

Windows命令行建议使用 `python -X utf8` 运行带中文或²单位的统计工具，避免默认GBK输出失败。测试使用下文discover命令，将host/tools加入正确的模块搜索路径，不用包形式导入旧测试。

[项目主页](../README.md) | 中文 | [English](README_EN.md)

工具与当前STM32串口协议v3配套。默认从PA9/USART1以460800 bit/s接收数据，并分别保存带GPS时间戳的IMU、1 Hz GNSS导航结果和1 Hz RAWX逐星逐频原始观测。

## 工具

多阶温补与标定的**唯一正式流程**见 [温补标定 SOP](../docs/温补标定SOP.md)：① `fit_temp_order_selection.m` → ① `fit_temp_bias_raw.m` → ② `calib24_static_numbered_tempcomp.m` → ③ `allan_compare_tc_configs.py`。步骤①②可用 `run_tempcal_sop.m` 一键跑通。理论背景见 [IMU温度补偿使用说明](../docs/IMU温度补偿使用说明.md)。

旧的 `allan_compare_before_after.py`、`fit_temp_bias_poly3.m` 及其标定域产物已从正式工具树删除。当前系数文件为 `temp_coeffs_raw.{mat,csv}`，阶数由拟合脚本的 `cfg.axisOrder` 决定；未拟合轴整行为 0。

| 文件 | 用途 | 默认输出 |
| --- | --- | --- |
| `capture_serial.py` | 无界面采集 | 会话文件夹中的 IMU、GNSS导航、RAWX CSV |
| `decode_rawx.py` | 校验并汇总 RAWX 原始观测 | 历元完整性、信号质量、锁定回退和相位-多普勒异常候选 |
| `check_rtcm_bridge.py` | 调用同一 Qt 代码短时检查 RTCM 链路 | 控制台统计，不生成文件 |
| `inspect_f9p.py` | 在 F9P 原生 USB 口只读查询 UBX 状态 | 控制台摘要，`--details` 显示逐信号状态 |
| `allan_noise_identification.py` | 直接读取标准 IMU CSV，辨识 Allan 随机误差 | `data/allan_results/` |
| `allan_compare_tc_configs.py` | 温补/标定三方案 Allan 对比（raw / raw+TC / raw+TC+calib） | `data/decoded/<session>/allan_compare_tc_configs/` |
| `run_tempcal_sop.m` | 一键跑通 SOP 步骤①（选阶+拟合系数）与步骤②（温补+24 位置标定） | `data/calib24/` 下全部温补标定产物 |
| `check_sync.py` | 核对 sync.csv 诊断文件完整性 | 控制台统计，不生成文件 |
| `analyze_turn_on_bias.py` | 手动上电采集后的温补标定、跨轮均值散布、质量检查和协方差 | 实验目录内的汇总、图表及中文报告；[使用说明](TURN_ON_BIAS.md) |
| `fit_temp_order_selection.m` | 逐轴1～5阶比较，推荐阶数仅供参考 | `temp_order_selection.*`、报告及图 |
| `fit_temp_bias_raw.m` | 原始域温补系数拟合；配置 `axisOrder` 是权威开关 | 温补MAT/CSV、全速率温补CSV及对照图 |
| `calib24_static_numbered_tempcomp.m` | 24位置逐样本先温补再标定 | 绑定温补配置的加计标定MAT、CSV及图 |
| `calib24_static_numbered.m` | 无温补24位置标定参考，不是当前test默认参数 | 原始域标定结果 |
| `gm_autocorrelation_analysis.m` | 温补标定后六轴ACF，10秒块平均传递函数修正 | 六轴GM候选参数CSV、块序列、MAT及自相关图 |
| `gm_validate_parameters.m` | 分段、去趋势、独立记录和30～600秒尺度复核 | 独立 `gm_validation/`，不覆盖原GM参数 |
| `position_to_kml.m` | PSINS位置按秒插值为KML，不需Mapping Toolbox | KML及返回的1 Hz位置/时间 |
| `tests/test_position_to_kml.m` | KML单位、XML、插值及边界回归 | 控制台通过/失败；临时测试文件自动清理 |

## 本版运行顺序与依赖（2026-09-30）

所有命令从仓库根目录执行。Python依赖：`python -m pip install -r tools/requirements_temp_analysis.txt`；采集另装 `tools/requirements.txt`。
MATLAB动态/静态INS脚本需要外部PSINS并先执行其路径初始化；温补、GM和KML工具不依赖PSINS。

1. 只复现电动车回放：直接使用已发布参数，按 [会话README](../data/decoded/20260923104556_电动车2/README.md) 运行test，无需重做21 h拟合。
2. 重新拟合：补齐21 h `imu.csv`，确认轴阶数后执行温补SOP；24位置01～24输入本版已发布。
3. GM辨识与默认独立验证需要另外补齐长时原始记录。仅查看已发布结果或重绘缓存不要求原始CSV，见 [GM说明](GM_VALIDATION.md)。
4. 上电重复性必须另采真正断电上电的数据；已发布三轮试跑仅为连续通电对照。

```matlab
addpath('tools');
run_tempcal_sop('skipStep1',true); % 复用冻结温补，重做24位置标定；会覆盖标定输出
% 以下两行需要补齐21 h和独立记录，不是下载后即可全流程重算：
% run('tools/gm_autocorrelation_analysis.m');
% results = gm_validate_parameters;
gm_validate_parameters(struct('replotOnly',true)); % 已发布验证MAT重绘
addpath('tools/tests'); test_position_to_kml;
```

### 辅助与历史工具

| 文件 | 目的/使用限制 |
| --- | --- |
| `analyze_latest_data.py` | 会话汇总质检；`python tools/analyze_latest_data.py <会话目录>`，不传目录会选本地最新会话 |
| `bnc_direct_proxy.py` | BNC本机直连代理；仅用于旧BNC链路，`--help`查看端口，Qt直连NTRIP不需要它 |
| `deep_dive_timing.py`、`innovation_dive.py` | 历史AIM时序/新息诊断；文件顶部硬编码本机目录，先修改并补齐记录再运行 |
| `extract_viz_data.py`、`gen_viz_html.py` | 历史下采样与HTML绘图辅助；先检查顶部数据/输出路径，不属于正式温补SOP |
| `test_capture_serial.py`、`test_decode_rawx.py` | `python -X utf8 -m unittest discover -s tools -p 'test_*.py'` |
| `requirements.txt`、`requirements_temp_analysis.txt` | 分别安装采集/Allan和温补统计依赖 |

参数MAT/CSV是冻结实验资产，重新运行拟合/分析会覆盖对应输出；换轴开关后必须整套重标定，不可混用。最新文件用途、上传范围与实验边界见 [版本指南](../docs/版本整理与使用指南_20260930.md)。

## 安装

`check_rtcm_bridge.py --stall-gui` 每十秒暂停 GUI 处理半秒，检查独立串口线程是否继续转发。输出重连次数、字节队列峰值、最长发送排队时间、MSM 不完整组丢弃计数及基站 MSM 头信息；可用 `--seconds 180` 做三分钟压力测试。MSM 的 `epoch_raw_ms` 保留原星座时间尺度：北斗转 GPS 需加 14 秒，GLONASS 字段为星期/日内毫秒组合，不能直接与 GPS 周内毫秒相减。测试摘要中的基站 ECEF 坐标是公开基站坐标，不输出流动站坐标。

`python tools/inspect_f9p.py COM3` 仅查询接收机原生 USB 的状态/配置，不写 VALSET、不复位、不注入 RTCM。`config_response_received=false` 表示本次没有收到配置查询响应，不能当作配置值为零；COM3 必须由设备枚举确认，不能用 COM7 替代。

`capture_serial.py` 在普通PC命令行模式下保持纯采集；树莓派服务显式配置
`--ntrip-control`后，可在同一串口所有者内建立NTRIP并处理`#RTCM`信用反馈。
Qt、树莓派采集器或其他串口程序仍不能同时打开同一个端口。

树莓派服务模式下，脚本持续发布实时位置，但只有检测到易失控制文件时才创建
和保存三个CSV；配置`--ubx-port`后还会同步保存F9P原始`f9p.ubx`。停止保存不
关闭串口。每条1 Hz GNSS记录会立即刷新，网页进程读取独立实时状态文件，不占用
串口。普通PC命令行用法仍默认立即保存。

关闭 Qt 后，可用 `python tools/check_rtcm_bridge.py COM7 --seconds 40 --bnc <你的私有配置路径>` 验证同一 Qt 转发代码。先停止 BNC 等其他差分注入源。省略 `--bnc` 时只读采集统计，不下发数据。测试不创建 CSV/raw 文件夹，不打印密码或位置坐标；输出接收、转发、丢帧、定位状态与 F9P 反馈，退出码 2 表示带基站测试未满足无丢帧、无错误且 F9P 收到数据的检查条件，不代表一定是串口故障。

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

树莓派GPIO5/RXD2旁路接收F9P UART1时，可增加第二串口参数：

    python3 tools/capture_serial.py /dev/ttyAMA0 --baud 460800 --ubx-port /dev/ttyAMA2 --ubx-baud 115200

此时同一会话中额外生成`f9p.ubx`。第二串口线程持续排空接收缓冲，但只有开始
采集后才落盘；文件按原始二进制字节保存，不进行文本解码或改写。

采集器默认每5秒输出一次实时状态，包括最近周期IMU频率、累计GNSS/RAWX/SAT、
丢帧数、无效行数和运行时间。可用`--status-interval 2`改为每2秒显示，或设为
0关闭周期显示。直接在终端运行时，状态会在同一行原位刷新；systemd后台运行
时可使用树莓派的`gnss-imu-status`命令查看同样的单行刷新状态。

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

### sync.csv 诊断文件（始终生成）

每次采集会话目录下**始终生成** `sync.csv`（每秒一行，独立于 `--save` 选择），记录 STM32 `# sync` 行的六个累计计数器、相对上一条 `#sync` 的四个增量，以及 backlog：

```text
unix_ms,pps,sample_count,interrupt_count,interrupt_overruns,cc2_overcapture,dt_gap_count,i2c_errors,d_interrupt_overruns,d_cc2_overcapture,d_dt_gap_count,d_i2c_errors,backlog
```

| 列 | 含义 |
| --- | --- |
| `unix_ms` | 本机接收该行时刻（毫秒） |
| `pps` | PPS 序号 |
| `sample_count` | 成功输出的 IMU 样本累计数 |
| `interrupt_count` | ISR 观察到的 DATA_RDY capture 累计数 |
| `interrupt_overruns` | 软件单槽 mailbox 被覆盖的累计数 |
| `cc2_overcapture` | TIM2 CCR2 硬件 overcapture 累计数 |
| `dt_gap_count` | `dt > 15 ms` 的 epoch 缺口累计数 |
| `i2c_errors` | I2C 读取失败累计数 |
| `d_*` 四个 | 相对上一条 `#sync` 的新增事件数（无符号 32 位差分） |
| `backlog` | `interrupt_count - sample_count`（无符号 32 位） |

语义要点：

- 四个 `d_*` **只表示相邻 `#sync` 之间新增的事件数**，不等于 lost samples；静止正常时均为 0。任一 `d_* > 0` 才表示上一秒新发生了对应异常。
- `backlog = interrupt_count - sample_count`：允许在 0/1 之间随 `#sync` 与主循环处理 DATA_RDY 的相位关系波动；需要关注的是**是否持续扩大**（如 1,1,2,3,4…），而不是单值是否为 1。
- 所有累计计数器是 32 位、可回绕，PC 端差分按无符号模 2³² 计算，长期运行也不会误报负数。

`check_sync.py` 只读核对 `sync.csv`：`python tools/check_sync.py <会话目录>/sync.csv`。

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

## GM 自相关参数复核

`gm_validate_parameters.m` 对现有GM参数做前后半/四分段、趋势敏感性、独立固定姿态记录和
30～600秒Allan尺度检验，绘图并输出表格，不覆盖现有GM参数。运行方式、默认独立窗口、
统计限制及test实际时间/姿态相关Q说明见 [GM_VALIDATION.md](GM_VALIDATION.md)。

## 位置导出Google Earth KML

`position_to_kml.m` 接收 `[纬度,经度,高度]` 和递增秒时间轴，默认角度为PSINS弧度、
高度为米，按整数秒插值为严格1 Hz，不外推。输出标准KML轨迹及起终点，无需Mapping Toolbox。

```matlab
addpath('tools');
[pos_1hz,t_1hz] = position_to_kml(avpL(:,7:9),avpL(:,10),'track_1hz.kml');
```

当前 `test.m` 已自动调用，使用每次传播并完成反馈后的天线端位置，而非仅有反馈时刻的日志。
输出为脚本同级 `kml/combined_navigation_1hz.kml`。在Google Earth中打开该文件即可查看轨迹。
默认 `AltitudeMode='clampToGround'` 贴地显示；坐标中仍保留高度。
若要显示HMSL海拔轨迹，改为 `'absolute'`；椭球高需先转换，不能直接当HMSL使用。
其他选项：`AngleUnit='deg'`、`SamplePeriod_s`、`Name`、`LineColor`（KML aabbggrr）、`LineWidth`。
返回的 `pos_1hz` 为 `[纬度deg,经度deg,高度m]`，`t_1hz` 与之逐行对应。
