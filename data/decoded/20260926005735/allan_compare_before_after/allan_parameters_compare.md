# 温漂补偿前后随机误差参数对比

- 补偿前：24 位标定补偿（未去温漂）
- 补偿后：24 位标定 + 3 阶温度模型温漂补偿
- 数据：imu_compensated.csv（7,605,132 样本）

| 传感器 | 轴 | 参数 | 补偿前 | 补偿后 | 变化 |
|---|---|---|---:|---:|---:|
| 加速度计 | X | ARW/VRW | 0.08699 m/s/sqrt(h) | 0.08699 m/s/sqrt(h) | 0.000765% |
| 加速度计 | X | BI | 0.04023 mg | 0.0434 mg | 7.87% |
| 加速度计 | X | RRW | 0.1432 mg/sqrt(h) | 0.1279 mg/sqrt(h) | -10.7% |
| 加速度计 | X | Rate Ramp | 未检出 mg/h | 未检出 mg/h | 未检出% |
| 加速度计 | Y | ARW/VRW | 0.0878 m/s/sqrt(h) | 0.0878 m/s/sqrt(h) | 1.69e-05% |
| 加速度计 | Y | BI | 0.02986 mg | 0.02997 mg | 0.352% |
| 加速度计 | Y | RRW | 0.04085 mg/sqrt(h) | 0.04121 mg/sqrt(h) | 0.887% |
| 加速度计 | Y | Rate Ramp | 未检出 mg/h | 未检出 mg/h | 未检出% |
| 加速度计 | Z | ARW/VRW | 0.1231 m/s/sqrt(h) | 0.1231 m/s/sqrt(h) | 0.0037% |
| 加速度计 | Z | BI | 0.07045 mg | 0.07047 mg | 0.0288% |
| 加速度计 | Z | RRW | 0.5339 mg/sqrt(h) | 0.4735 mg/sqrt(h) | -11.3% |
| 加速度计 | Z | Rate Ramp | 0.7293 mg/h | 1.107 mg/h | 51.9% |
| 陀螺仪 | X | ARW/VRW | 0.1917 deg/sqrt(h) | 0.1925 deg/sqrt(h) | 0.421% |
| 陀螺仪 | X | BI | 10.33 deg/h | 9.824 deg/h | -4.93% |
| 陀螺仪 | X | RRW | 89.29 deg/h/sqrt(h) | 34.15 deg/h/sqrt(h) | -61.8% |
| 陀螺仪 | X | Rate Ramp | 0.03307 deg/s per h | 未检出 deg/s per h | 未检出% |
| 陀螺仪 | Y | ARW/VRW | 0.2187 deg/sqrt(h) | 0.2187 deg/sqrt(h) | 0.00859% |
| 陀螺仪 | Y | BI | 9.386 deg/h | 9.234 deg/h | -1.62% |
| 陀螺仪 | Y | RRW | 42.16 deg/h/sqrt(h) | 26.59 deg/h/sqrt(h) | -36.9% |
| 陀螺仪 | Y | Rate Ramp | 0.01038 deg/s per h | 未检出 deg/s per h | 未检出% |
| 陀螺仪 | Z | ARW/VRW | 0.2536 deg/sqrt(h) | 0.2536 deg/sqrt(h) | 0.00402% |
| 陀螺仪 | Z | BI | 4.514 deg/h | 4.926 deg/h | 9.11% |
| 陀螺仪 | Z | RRW | 19.16 deg/h/sqrt(h) | 24.9 deg/h/sqrt(h) | 30% |
| 陀螺仪 | Z | Rate Ramp | 0.006562 deg/s per h | 0.009134 deg/s per h | 39.2% |
