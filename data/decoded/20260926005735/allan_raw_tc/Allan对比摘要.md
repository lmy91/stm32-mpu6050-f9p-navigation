# Allan 对比摘要

- 原始数据：`imu.csv`
- 温补数据：`imu_tempcomp_multiorder.csv`
- 样本数：7,605,132；时长：21.14 h；采样率：99.9477 Hz
- 计算阶段：Raw, TC
- 图片：`allan_raw_tc_1800x780.png`（1800×780 px）

## 600 s Allan 偏差

| 轴 | 单位 | Raw | TC |
|---|---|---:|---:|
| AX | mg | 0.02853 | 0.03666 |
| AY | mg | 0.01992 | 0.01994 |
| AZ | mg | 0.1603 | 0.1374 |
| GX | deg/h | 24.19 | 8.657 |
| GY | deg/h | 10.2 | 8.836 |
| GZ | deg/h | 4.402 | 5.634 |
