# MATLAB工具回归测试

[工具说明](../README.md)

`test_position_to_kml.m` 验证1 Hz整数秒网格、插值、弧度/度、KML经纬高顺序、
UTF-8/XML转义、日期变更线、单点输出和非法输入处理。
无需PSINS或Mapping Toolbox，但XML测试使用MATLAB的Java XML读取支持。

在仓库根目录MATLAB执行：

```matlab
addpath('tools/visualization/tests');
test_position_to_kml;
```

通过会打印passed，失败会抛异常。测试在新建临时目录写KML，结束后清理该目录，
不覆盖实验KML或修改原始记录。
