"""Generate self-contained HTML visualization with ECharts."""
import csv
import json
import pathlib

DATA_DIR = pathlib.Path(r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260909203315")
OUT = pathlib.Path(r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260909203315\analysis_report.html")

def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def ds(rows, maxp):
    if len(rows) <= maxp:
        return rows
    step = max(1, len(rows) // maxp)
    return rows[::step]

# AIM
aim = load_csv(DATA_DIR / "aim.csv")
aim = ds(aim, 200)
at = [round(float(r["elapsed_s"]), 2) for r in aim]
aroll = [round(float(r["roll_deg"]), 3) for r in aim]
apitch = [round(float(r["pitch_deg"]), 3) for r in aim]
ahead = [round(float(r["heading_deg"]), 3) for r in aim]
apn = [round(float(r["pos_n_m"]), 2) for r in aim]
ape = [round(float(r["pos_e_m"]), 2) for r in aim]
apd = [round(float(r["pos_d_m"]), 2) for r in aim]
avn = [round(float(r["vel_n_m_s"]), 3) for r in aim]
ave = [round(float(r["vel_e_m_s"]), 3) for r in aim]
avd = [round(float(r["vel_d_m_s"]), 3) for r in aim]
asr = [round(float(r["std_att_roll_deg"]), 4) for r in aim]
asp = [round(float(r["std_att_pitch_deg"]), 4) for r in aim]
ash = [round(float(r["std_att_heading_deg"]), 4) for r in aim]
aspn = [round(float(r["std_pos_n_m"]), 3) for r in aim]
aspe = [round(float(r["std_pos_e_m"]), 3) for r in aim]
aspd = [round(float(r["std_pos_d_m"]), 3) for r in aim]

# GNSS
gnss = load_csv(DATA_DIR / "gnss.csv")
gt0 = int(gnss[0]["gps_tow_ms"])
gt = [round((int(r["gps_tow_ms"]) - gt0) / 1000.0, 1) for r in gnss]
ghacc = [round(float(r["h_acc_m"]), 3) for r in gnss]
gvacc = [round(float(r["v_acc_m"]), 3) for r in gnss]
gpdop = [round(float(r["pdop"]), 2) for r in gnss]
gsv = [int(r["num_sv"]) for r in gnss]
gspd = [round(float(r["ground_speed_m_s"]), 4) for r in gnss]

# IMU
imu = load_csv(DATA_DIR / "imu.csv")
imu = ds(imu, 300)
it = [round(float(r["time_s"]), 2) for r in imu]
iax = [round(float(r["ax_m_s2"]), 4) for r in imu]
iay = [round(float(r["ay_m_s2"]), 4) for r in imu]
iaz = [round(float(r["az_m_s2"]), 4) for r in imu]
igx = [round(float(r["gx_deg_h"]) / 3600, 4) for r in imu]
igy = [round(float(r["gy_deg_h"]) / 3600, 4) for r in imu]
igz = [round(float(r["gz_deg_h"]) / 3600, 4) for r in imu]

html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>采集数据分析报告 - 20260910142516</title>
<script src="https://cdn.jsdelivr.net/npm/echarts@5.5.0/dist/echarts.min.js"></script>
<style>
body {{ font-family: -apple-system, "Microsoft YaHei", sans-serif; background: #0d1117; color: #e6edf3; margin: 0; padding: 20px; }}
h1 {{ font-size: 22px; border-bottom: 2px solid #30363d; padding-bottom: 10px; }}
h2 {{ font-size: 17px; color: #58a6ff; margin-top: 30px; }}
.card {{ background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 15px; margin: 15px 0; }}
.chart {{ width: 100%; height: 320px; }}
.grid2 {{ display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }}
.warn {{ color: #f85149; font-weight: bold; }}
.ok {{ color: #3fb950; font-weight: bold; }}
.note {{ color: #8b949e; font-size: 13px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px; }}
th, td {{ border: 1px solid #30363d; padding: 6px 10px; text-align: left; }}
th {{ background: #21262d; }}
</style>
</head>
<body>
<h1>导航采集数据分析报告 — 2026-09-10 14:25:16</h1>
<p class="note">数据来源：data/decoded/20260910142516/ · 采集时长 9.8分钟 · IMU 58873帧 / GNSS 589历元 / AIM 3793行</p>

<div class="card">
<h2>一、AIM 融合解算 — 位置漂移（核心问题）</h2>
<p class="warn">精对准阶段位置漂移达 396.6m（E方向391m + N方向65m），速度峰值 35.8 m/s，滤波器发散。</p>
<div id="chart_pos" class="chart"></div>
</div>

<div class="card">
<h2>二、AIM 融合解算 — 速度发散</h2>
<p class="warn">Ve方向速度出现高达35.8 m/s的尖峰，而GNSS实测地速<0.11 m/s（静止）。21次GNSS更新因"速度新息超限"被拒绝。</p>
<div id="chart_vel" class="chart"></div>
</div>

<div class="card">
<h2>三、AIM 姿态收敛</h2>
<p>Roll稳定在-0.9°±0.38°；Heading被约束在90°±0.06°；<span class="warn">Pitch从-2.09°发散至-14.3°（变化12.2°）</span>。</p>
<div id="chart_att" class="chart"></div>
</div>

<div class="card">
<h2>四、AIM 协方差收敛（1σ）</h2>
<p>姿态1σ从2°收敛到0.06-0.07°，航向从20°收敛到0.12°；位置1σ收敛到2.5-2.8m（水平）/0.66m（垂直）。协方差本身收敛，但状态估计已发散。</p>
<div id="chart_std" class="chart"></div>
</div>

<div class="grid2">
<div class="card">
<h2>五、GNSS 质量指标</h2>
<p class="ok">100% 3D固定解，平均24.2颗星，PDOP均值1.49。</p>
<p class="warn">carr_soln=0（全部历元无RTK），diff_soln=0，RTCM差分未生效。</p>
<div id="chart_gnss" class="chart"></div>
</div>
<div class="card">
<h2>六、GNSS 静止验证</h2>
<p class="ok">地速RMS=0.029 m/s，2D位置RMS=1.46m（单点定位正常水平）。确认设备静止。</p>
<div id="chart_gnss2" class="chart"></div>
</div>
</div>

<div class="card">
<h2>七、IMU 原始数据（降采样）</h2>
<p>加速度计Az均值-10.35 m/s²（偏G0 5.6%），陀螺零偏X=-8.36°/s、Y=-2.20°/s、Z=-1.63°/s。存在瞬态尖峰。</p>
<div id="chart_imu_acc" class="chart"></div>
<div id="chart_imu_gyro" class="chart"></div>
</div>

<div class="card">
<h2>八、数据质量总览</h2>
<table>
<tr><th>指标</th><th>数值</th><th>判定</th></tr>
<tr><td>IMU采样率</td><td>99.95 Hz（目标100Hz）</td><td class="ok">合理</td></tr>
<tr><td>IMU丢帧</td><td>0帧</td><td class="ok">合理</td></tr>
<tr><td>IMU dt抖动</td><td>max 0.001ms</td><td class="ok">优秀</td></tr>
<tr><td>GNSS历元间隔</td><td>1.000s（无间隙）</td><td class="ok">合理</td></tr>
<tr><td>GNSS定位状态</td><td>100% 3D固定</td><td class="ok">合理</td></tr>
<tr><td>RTK状态</td><td>0%固定/浮点（无差分）</td><td class="warn">异常-RTCM未生效</td></tr>
<tr><td>hAcc均值</td><td>0.746m（单点定位）</td><td class="ok">合理</td></tr>
<tr><td>加速度计Az偏差</td><td>-10.35 vs -9.81（+5.6%）</td><td class="warn">偏大</td></tr>
<tr><td>陀螺X零偏</td><td>-8.36°/s</td><td class="warn">偏大（规格内）</td></tr>
<tr><td>AIM位置漂移</td><td>396.6m / 353s</td><td class="warn">发散</td></tr>
<tr><td>AIM速度峰值</td><td>35.8 m/s（Ve）</td><td class="warn">发散</td></tr>
<tr><td>AIM Pitch变化</td><td>12.2°（-2.1°→-14.3°）</td><td class="warn">发散</td></tr>
<tr><td>GNSS更新接受率</td><td>94.1%（332/353）</td><td class="ok">尚可</td></tr>
<tr><td>STM32定时器漂移</td><td>70.6 ppm</td><td class="ok">合理</td></tr>
</table>
</div>

<script>
const dark = {{ backgroundColor: 'transparent', textStyle: {{ color: '#e6edf3' }} }};
const axisStyle = {{ axisLine: {{ lineStyle: {{ color: '#30363d' }} }}, axisLabel: {{ color: '#8b949e' }}, splitLine: {{ lineStyle: {{ color: '#21262d' }} }} }};

function mk(id, option) {{
  const c = echarts.init(document.getElementById(id), 'dark');
  c.setOption(option);
  window.addEventListener('resize', () => c.resize());
}}

mk('chart_pos', {{
  ...dark, title: {{ text: 'NED位置（相对原点）', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['N','E','D'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 60, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(at)}, name: 'elapsed(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: '位置(m)', ...axisStyle }},
  series: [
    {{ name: 'N', type: 'line', data: {json.dumps(apn)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'E', type: 'line', data: {json.dumps(ape)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'D', type: 'line', data: {json.dumps(apd)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
  ]
}});

mk('chart_vel', {{
  ...dark, title: {{ text: 'NED速度', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['Vn','Ve','Vd'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 60, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(at)}, name: 'elapsed(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: '速度(m/s)', ...axisStyle }},
  series: [
    {{ name: 'Vn', type: 'line', data: {json.dumps(avn)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'Ve', type: 'line', data: {json.dumps(ave)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'Vd', type: 'line', data: {json.dumps(avd)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
  ]
}});

mk('chart_att', {{
  ...dark, title: {{ text: '姿态角 Roll/Pitch/Heading', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['Roll','Pitch','Heading'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 60, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(at)}, name: 'elapsed(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: '角度(°)', ...axisStyle }},
  series: [
    {{ name: 'Roll', type: 'line', data: {json.dumps(aroll)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'Pitch', type: 'line', data: {json.dumps(apitch)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'Heading', type: 'line', data: {json.dumps(ahead)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
  ]
}});

mk('chart_std', {{
  ...dark, title: {{ text: '协方差1σ — 姿态与位置', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['σRoll','σPitch','σHeading','σPosN','σPosE','σPosD'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 60, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(at)}, name: 'elapsed(s)', ...axisStyle }},
  yAxis: [
    {{ type: 'value', name: '姿态(°)', ...axisStyle }},
    {{ type: 'value', name: '位置(m)', ...axisStyle, splitLine: {{ show: false }} }}
  ],
  series: [
    {{ name: 'σRoll', type: 'line', data: {json.dumps(asr)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'σPitch', type: 'line', data: {json.dumps(asp)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'σHeading', type: 'line', data: {json.dumps(ash)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'σPosN', type: 'line', yAxisIndex: 1, data: {json.dumps(aspn)}, lineStyle: {{ width: 1.5, type: 'dashed' }}, symbol: 'none' }},
    {{ name: 'σPosE', type: 'line', yAxisIndex: 1, data: {json.dumps(aspe)}, lineStyle: {{ width: 1.5, type: 'dashed' }}, symbol: 'none' }},
    {{ name: 'σPosD', type: 'line', yAxisIndex: 1, data: {json.dumps(aspd)}, lineStyle: {{ width: 1.5, type: 'dashed' }}, symbol: 'none' }},
  ]
}});

mk('chart_gnss', {{
  ...dark, title: {{ text: 'GNSS精度与星座', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['hAcc','vAcc','PDOP','卫星数'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 55, right: 55, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(gt)}, name: '时间(s)', ...axisStyle }},
  yAxis: [
    {{ type: 'value', name: '精度(m)/PDOP', ...axisStyle }},
    {{ type: 'value', name: '卫星数', ...axisStyle, splitLine: {{ show: false }} }}
  ],
  series: [
    {{ name: 'hAcc', type: 'line', data: {json.dumps(ghacc)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'vAcc', type: 'line', data: {json.dumps(gvacc)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: 'PDOP', type: 'line', data: {json.dumps(gpdop)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
    {{ name: '卫星数', type: 'line', yAxisIndex: 1, data: {json.dumps(gsv)}, lineStyle: {{ width: 1.5 }}, symbol: 'none' }},
  ]
}});

mk('chart_gnss2', {{
  ...dark, title: {{ text: 'GNSS地速（静止验证）', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  grid: {{ left: 55, right: 20, top: 50, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(gt)}, name: '时间(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: '地速(m/s)', ...axisStyle }},
  series: [
    {{ name: '地速', type: 'line', data: {json.dumps(gspd)}, lineStyle: {{ width: 1.5, color: '#3fb950' }}, symbol: 'none', areaStyle: {{ color: 'rgba(63,185,80,0.15)' }} }}
  ]
}});

mk('chart_imu_acc', {{
  ...dark, title: {{ text: 'IMU加速度计 (m/s²)', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['Ax','Ay','Az'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 55, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(it)}, name: 'time(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: 'm/s²', ...axisStyle }},
  series: [
    {{ name: 'Ax', type: 'line', data: {json.dumps(iax)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
    {{ name: 'Ay', type: 'line', data: {json.dumps(iay)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
    {{ name: 'Az', type: 'line', data: {json.dumps(iaz)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
  ]
}});

mk('chart_imu_gyro', {{
  ...dark, title: {{ text: 'IMU陀螺仪 (deg/s)', left: 'center', textStyle: {{ fontSize: 14 }} }},
  tooltip: {{ trigger: 'axis', triggerOn: 'click', renderMode: 'richText', confine: true }},
  legend: {{ data: ['Gx','Gy','Gz'], top: 25, textStyle: {{ color: '#8b949e' }} }},
  grid: {{ left: 55, right: 20, top: 60, bottom: 40 }},
  xAxis: {{ type: 'category', data: {json.dumps(it)}, name: 'time(s)', ...axisStyle }},
  yAxis: {{ type: 'value', name: 'deg/s', ...axisStyle }},
  series: [
    {{ name: 'Gx', type: 'line', data: {json.dumps(igx)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
    {{ name: 'Gy', type: 'line', data: {json.dumps(igy)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
    {{ name: 'Gz', type: 'line', data: {json.dumps(igz)}, lineStyle: {{ width: 1 }}, symbol: 'none' }},
  ]
}});
</script>
</body>
</html>"""

OUT.write_text(html, encoding="utf-8")
print(f"Written {OUT} ({OUT.stat().st_size:,} bytes)")
