# 温补与24位置标定

[工具索引](../README.md) | [温补标定 SOP](../../docs/温补标定SOP.md)

正式流程依次为 `fit_temp_order_selection.m`（选阶参考）、`fit_temp_bias_raw.m`（按配置拟合）、`calib24_static_numbered_tempcomp.m`（先温补再标定）。`run_tempcal_sop.m` 一键执行上述步骤。`calib24_static_numbered.m` 是不温补的参考标定。

在仓库根目录的 MATLAB 执行：

```matlab
addpath('tools'); setup_tools;
run_tempcal_sop('skipStep1',true); % 复用已发布温补系数，重做标定
```

输入和输出位于 `data/calib24/`。24位置输入已发布；重拟合温补还需另行补齐21小时原始记录。重跑会覆盖相应标定产物，轴配置与系数、标定结果必须配套。后续三方案 Allan 对比位于 `tools/noise_analysis/`。
