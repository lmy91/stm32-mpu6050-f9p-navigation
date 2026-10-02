# Allan 与 GM 分析

[工具索引](../README.md) | [GM 参数复核](GM_VALIDATION.md)

| 文件 | 用途 |
| --- | --- |
| `allan_noise_identification.py` | 标准 IMU CSV 的 Allan 随机误差辨识 |
| `allan_compare_tc_configs.py` | raw / TC / TC+calibration 对比，复用同目录算法库 |
| `gm_autocorrelation_analysis.m` | 六轴自相关与候选 GM 参数辨识 |
| `gm_validate_parameters.m` | 分段、去趋势、独立记录与多时间尺度复核 |

```powershell
python -m pip install -r tools/requirements_temp_analysis.txt
python -X utf8 tools/noise_analysis/allan_noise_identification.py --help
python -X utf8 tools/noise_analysis/allan_compare_tc_configs.py --help
```

在仓库根目录的 MATLAB 中初始化并重绘已发布结果：

```matlab
addpath('tools'); setup_tools;
gm_validate_parameters(struct('replotOnly',true));
```

长时原始记录未收入仓库，完整重算与默认独立验证需补齐输入。输出位于 `data/` 内对应结果目录；候选参数及统计边界见 GM 说明。
