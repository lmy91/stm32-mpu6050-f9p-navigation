# 零偏上电重复性

[工具索引](../README.md) | [完整采集与分析方法](TURN_ON_BIAS.md)

`analyze_turn_on_bias.py` 分析手动采集的静态会话，执行固定温补/标定、质量检查和跨轮均值协方差统计。

```powershell
python -m pip install -r tools/requirements_temp_analysis.txt
python -X utf8 tools/repeatability/analyze_turn_on_bias.py --help
```

命令从仓库根目录执行，结果写入指定实验的 `analysis/`。工具不控制电源或设备。已发布三轮试跑属于连续通电对照，正式重复性需要另外采集真实断电上电周期。
