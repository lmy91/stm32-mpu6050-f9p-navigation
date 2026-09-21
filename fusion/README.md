# PC 实时对准与 GNSS/INS 松组合

中文 | [English](README_EN.md)

`realtime_self_aim.py` 完成 PC 端粗/精对准，`realtime_loose_navigation.py` 接续精对准末状态执行实时 GNSS/INS 松组合。两者直接复用串口协议 v3 的同步数据：100 Hz IMU 驱动 NED 惯导递推，1 Hz F9P WGS-84 位置和 NED 速度进行15状态闭环误差滤波。Qt“自瞄”页负责对准，“组合导航”页显示组合位置地图、NED速度与FRD姿态时间序列；连接前勾选 `AIM`/`NAV` 可分别保存 `aim.csv`/`nav.csv`。

当前流程为：配置装订 → 粗对准 → 精对准 → 点击“结束精对准并启动组合导航”。粗对准达到配置时长后自动切换，也可由Qt按钮手动切换；精对准持续运行，直到用户手动转入组合导航或停止。转入时完整复制末时刻AVP、零偏、P阵、GM参考值和GPS时刻，下一颗IMU继续递推。具体参数由 [`self_aim_config.json`](self_aim_config.json) 或Qt“算法配置”控制。

当前IMU噪声已采用 `data/allan_results/imu_noise/allan_parameters.csv` 的最新长时静态标定结果。陀螺ARW与加计VRW分别作为角速度、比力白噪声密度；陀螺和加计零偏均采用分轴一阶高斯-马尔可夫模型：

```text
δb(k+1) = exp(-dt/τ) δb(k) + w(k)
Qb = σb² [1 - exp(-2dt/τ)]
```

`gyro_bias_sigma_deg_h` 和 `accel_bias_sigma_mg` 取Allan零偏不稳定性作为GM稳态尺度；初始P阵的零偏标准差由独立的 `initial_*_bias_std_*` 配置。`*_bias_correlation_time_s` 暂取各轴零偏平台拟合区间的几何中心。名义零偏围绕粗对准估计的启机零偏作相关变化，而不是被拉向绝对零。由于本次长周期数据存在超过2°C温变，未将RRW/Rate Ramp用于过程噪声；相关时间是可复现实验初值，后续应以恒温数据或零偏自相关函数复核。

量测R阵支持两种模式。实时模式每历元使用F9P精度构造 `σpos=[hAcc,hAcc,vAcc]`、`σvel=[sAcc,sAcc,sAcc]`；某个实时值为零或非法时只回退该类默认值。固定模式直接使用配置的默认三轴位置/速度标准差。

## 使用

1. 先在配置中确认 `body_from_sensor`。它把 MPU6050 传感器坐标转换到载体坐标；默认单位阵仅适用于传感器轴与载体前-右-下轴一致的安装。
2. 将 `lever_arm_body_m` 改为 IMU 到 F9P 天线相位中心的载体系杆臂，单位 m；默认 `[0,0,0]`。
3. MPU6050不能可靠静态寻北，因此 `initial_heading_deg` 只允许人工装订，程序不会再用GNSS航迹自动覆盖它。`heading_measurement_std_deg` 表示精对准航向约束的1σ。
4. 启动Qt并连接串口，在“算法配置…”完成参数和航向装订。点击“开始粗对准”；达到 `coarse_alignment_seconds`（且样本与GNSS有效）或点击“立即进入精对准”后切换。精对准一直运行，点击“停止”结束。
5. 粗对准转精对准时，姿态取当前粗对准结果，速度设为 `[0,0,0]`，位置取粗对准期间所有质量合格GNSS位置的算术均值。精对准稳定后在“组合导航”页点击启动；`aim.csv` 保存对准全过程，`nav.csv` 默认100 Hz保存实时组合位置、速度、姿态、零偏、标准差和延迟重放诊断。同一GPS时刻只保存最终校正结果，`output_source` 标记惯推或GNSS重放修正；同目录 `session.json` 保存本次实际生效的完整参数，便于复现。

配置加载执行字段、数值、旋转矩阵正交性检查；GNSS解无效、时间差、hAcc/sAcc/PDOP超限时拒绝当前历元，但不会停止惯导递推。组合导航线程保存 `navigation_buffer_seconds` 长度的IMU及每步状态快照。GNSS晚到时恢复到真实 `t_gnss` 状态，执行位置/速度更新，再按缓存IMU逐点重放到当时最新IMU时刻；串口线程仍可同时接收新数据，排队后继续处理。超过 `maximum_gnss_age_s` 或早于缓存的量测会拒绝。当前不设置位置/速度组合新息门限。

精对准采用闭环误差状态反馈。零偏误差定义为“当前估计零偏−真实零偏”；每次有效GNSS量测更新后执行 `b_g ← b_g-δb_g`、`b_a ← b_a-δb_a`，随后将15维误差状态清零。下一颗IMU样本立即使用 `ω=ω_m-b_g`、`f=f_m-b_a`，因此估计出的传感器零偏会实际补偿后续姿态、速度和位置递推；原始IMU采集文件不被改写。

## 坐标系与线程

导航系固定为北-东-地（NED），速度和局部位置均按 N/E/D 输出；载体系固定为前-右-下（FRD），姿态为FRD载体系相对NED导航系的横滚/俯仰/航向。`body_from_sensor` 明确保留为“IMU传感器坐标 → FRD载体系”的3×3方向余弦矩阵，对准与组合导航共用，数据文件中的原始IMU坐标不被改写。

串口线程只收发字节，Qt线程完成协议解析和显示；组合导航由单独线程独占可变INS/KF状态。解析后的带GPS时间戳IMU/GNSS对象送入有界队列，组合线程完成传播、延迟量测更新和重放后，通过Qt信号返回只读结果。地图与CSV写入仍在Qt线程，不会并发修改滤波器。

## 当前边界

这是用于联调和后续算法迭代的松组合原型，不是飞行认证软件。当前为局部 NED 15 状态误差滤波，已实现按GNSS历元回退/重放和一阶杆臂速度近似，但不包含长距离局部原点重置、双子样锥划/划船补偿、比例因子/安装角在线估计、故障隔离冗余或紧组合原始观测滤波。正式试验前必须标定 MPU6050 零偏/比例因子/安装角、实测时间延迟和杆臂，并用参考轨迹验证。

自动测试：

    D:\anaconda\envs\allan-toolkit\python.exe -m unittest fusion.test_realtime_self_aim fusion.test_realtime_loose_navigation -v
