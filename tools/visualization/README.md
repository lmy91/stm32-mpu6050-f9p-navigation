# 轨迹导出

[工具索引](../README.md) | [测试说明](tests/README.md)

`position_to_kml.m` 将位置按整数秒插值为 KML，默认输入纬经度为弧度、高度为米，无需 Mapping Toolbox。输出位置为角度，不外推。

在仓库根目录的 MATLAB 执行：

```matlab
addpath('tools'); setup_tools;
[pos_1hz,t_1hz] = position_to_kml(llh_rad,time_s,'track_1hz.kml');
addpath('tools/visualization/tests'); test_position_to_kml;
```

默认贴地显示；`'AltitudeMode','absolute'` 需要海拔高度。电动车回放已经直接加载本目录并导出到会话的 `kml/`。
