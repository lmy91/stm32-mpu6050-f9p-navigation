"use strict";

const $ = (id) => document.getElementById(id);
const elements = {
  positionState: $("positionState"), recordingState: $("recordingState"), ntripState: $("ntripState"),
  startRecording: $("startRecording"), stopRecording: $("stopRecording"), recordHint: $("recordHint"),
  ntripConnect: $("ntripConnect"), ntripDisconnect: $("ntripDisconnect"), ntripHint: $("ntripHint"),
  fix: $("fixText"), satellites: $("satellites"), pdop: $("pdop"), speed: $("speed"), hAcc: $("hAcc"),
  latitude: $("latitude"), longitude: $("longitude"), height: $("height"), velN: $("velN"), velE: $("velE"),
  velD: $("velD"), gpsTime: $("gpsTime"), age: $("age"), session: $("session"), message: $("message"),
  count: $("pointCount"), empty: $("mapEmpty"), canvas: $("trackCanvas"), amap: $("amap"),
  skyCanvas: $("skyCanvas"), skyCount: $("skyCount"), skyAge: $("skyAge"),
  imuState: $("imuState"), imuAge: $("imuAge"),
  imuAx: $("imuAx"), imuAy: $("imuAy"), imuAz: $("imuAz"),
  imuGx: $("imuGx"), imuGy: $("imuGy"), imuGz: $("imuGz"), imuTemp: $("imuTemp"),
  accelCanvas: $("accelCanvas"), gyroCanvas: $("gyroCanvas"), temperatureCanvas: $("temperatureCanvas"),
  commandForm: $("commandForm"), commandInput: $("commandInput"),
  commandRun: $("commandRun"), commandOutput: $("commandOutput")
};

let points = [], latestKey = "", latestData = null;
let recordingMode = false, recordingSession = null, localMode = true;
let amapInstance = null, amapMarker = null, amapDots = [], amapLoading = false;
let skySatellites = [];
let imuHistory = [], latestImuKey = "";
let commandHistory = [], commandHistoryIndex = 0;
const IMU_HISTORY_SECONDS = 120;
const axisColors = {x:"#3fa7ff", y:"#ffb84d", z:"#45e2a0", temp:"#e879f9"};
const constellationStyle = {
  0: {prefix:"G", color:"#3fa7ff"}, 1: {prefix:"S", color:"#9aa7b8"},
  2: {prefix:"E", color:"#ff6178"}, 3: {prefix:"B", color:"#ffb84d"},
  5: {prefix:"Q", color:"#b785ff"}, 6: {prefix:"R", color:"#45e2a0"},
  7: {prefix:"I", color:"#e879f9"}
};

function fixed(value, digits = 3, suffix = "") {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
}
function pointKey(item) { return `${item.gps_week}:${item.gps_tow_ms}`; }
function pointColor(item) { return item.carr_soln === 2 ? "#45e2a0" : item.carr_soln === 1 ? "#ffb84d" : "#2bd9fe"; }
function gpsLabel(data) {
  return Number.isFinite(data.gps_week) && Number.isFinite(data.gps_tow_ms)
    ? `W${data.gps_week} ${(data.gps_tow_ms / 1000).toFixed(3)}s` : "--";
}
function validPoint(item) {
  return item && Number.isFinite(item.lat_deg) && Number.isFinite(item.lon_deg) &&
    Math.abs(item.lat_deg) <= 90 && Math.abs(item.lon_deg) <= 180;
}

function wgs84ToGcj02(lat, lon) {
  if (lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271) return [lat, lon];
  const pi = Math.PI, a = 6378245.0, ee = 0.006693421622965943;
  const transformLat = (x, y) => {
    let r = -100 + 2*x + 3*y + .2*y*y + .1*x*y + .2*Math.sqrt(Math.abs(x));
    r += (20*Math.sin(6*x*pi) + 20*Math.sin(2*x*pi))*2/3;
    r += (20*Math.sin(y*pi) + 40*Math.sin(y*pi/3))*2/3;
    return r + (160*Math.sin(y*pi/12) + 320*Math.sin(y*pi/30))*2/3;
  };
  const transformLon = (x, y) => {
    let r = 300 + x + 2*y + .1*x*x + .1*x*y + .1*Math.sqrt(Math.abs(x));
    r += (20*Math.sin(6*x*pi) + 20*Math.sin(2*x*pi))*2/3;
    r += (20*Math.sin(x*pi) + 40*Math.sin(x*pi/3))*2/3;
    return r + (150*Math.sin(x*pi/12) + 300*Math.sin(x*pi/30))*2/3;
  };
  let dLat = transformLat(lon - 105, lat - 35), dLon = transformLon(lon - 105, lat - 35);
  const rad = lat / 180 * pi, magic = 1 - ee * Math.sin(rad) ** 2, root = Math.sqrt(magic);
  dLat = dLat * 180 / ((a * (1-ee) / (magic * root)) * pi);
  dLon = dLon * 180 / ((a / root * Math.cos(rad)) * pi);
  return [lat + dLat, lon + dLon];
}

function clearAmapDots() {
  if (amapInstance && amapDots.length) amapInstance.remove(amapDots.map(item => item.overlay));
  amapDots = [];
}
function addAmapDot(item) {
  if (!amapInstance || !validPoint(item)) return;
  const key = pointKey(item);
  if (amapDots.length && amapDots[amapDots.length - 1].key === key) return;
  const converted = wgs84ToGcj02(item.lat_deg, item.lon_deg);
  const overlay = new AMap.CircleMarker({center: [converted[1], converted[0]], radius: 3.5,
    strokeOpacity: 0, fillColor: pointColor(item), fillOpacity: .9, zIndex: 10, map: amapInstance});
  amapDots.push({key, overlay});
  if (amapDots.length > 600) amapInstance.remove(amapDots.shift().overlay);
}
function updateAmapCurrent() {
  if (!amapInstance || !validPoint(latestData)) return;
  const converted = wgs84ToGcj02(latestData.lat_deg, latestData.lon_deg), position = [converted[1], converted[0]];
  if (!amapMarker) amapMarker = new AMap.CircleMarker({center: position, radius: 7,
    strokeColor: "#ffffff", strokeWeight: 2, fillColor: pointColor(latestData),
    fillOpacity: 1, zIndex: 30, map: amapInstance});
  else {
    amapMarker.setCenter(position);
    amapMarker.setOptions({fillColor: pointColor(latestData)});
  }
  if (points.length < 3) amapInstance.setCenter(position);
}

function refreshPointLabels() {
  elements.count.textContent = recordingMode ? `采集轨迹 ${points.length} 点` : `当前位置 ${points.length ? 1 : 0} 点`;
  elements.empty.hidden = points.length > 0;
}
function replacePoints(items) {
  points = items.filter(validPoint).slice(-5000);
  latestKey = points.length ? pointKey(points[points.length - 1]) : "";
  clearAmapDots();
  if (amapInstance) points.slice(-600).forEach(addAmapDot);
  refreshPointLabels(); drawLocalTrack(); updateAmapCurrent();
}
function appendPoint(item) {
  if (!validPoint(item) || pointKey(item) === latestKey) return;
  latestKey = pointKey(item); points.push(item);
  if (points.length > 5000) points.shift();
  addAmapDot(item); refreshPointLabels(); drawLocalTrack(); updateAmapCurrent();
}
async function loadRecordingHistory(session) {
  if (!session) return;
  try {
    const query = new URLSearchParams({limit: "600", session});
    const response = await fetch(`/api/track?${query}`, {cache: "no-store"}), payload = await response.json();
    if (!payload.ok || !recordingMode || recordingSession !== session || payload.session !== session) return;
    replacePoints(payload.points); if (latestData) appendPoint(latestData);
  } catch (_) { /* regular polling reports connectivity */ }
}
function syncPoints(payload, data) {
  latestData = data;
  const active = Boolean(payload.recording), session = active ? payload.session : null;
  if (!active) {
    recordingMode = false; recordingSession = null;
    if (points.length !== 1 || latestKey !== pointKey(data)) replacePoints([data]);
    else updateAmapCurrent();
    return;
  }
  const changed = !recordingMode || recordingSession !== session;
  recordingMode = true; recordingSession = session;
  if (changed) { replacePoints([data]); loadRecordingHistory(session); }
  else appendPoint(data);
}

function updateNtrip(value, serviceRunning) {
  const ntrip = value || {requested: false, connected: false, phase: "未连接", error: ""};
  elements.ntripState.className = `live-state ${ntrip.connected ? "online" : ntrip.requested ? "" : "idle"}`;
  elements.ntripState.lastElementChild.textContent = `基站：${ntrip.phase || (ntrip.requested ? "连接中" : "未连接")}`;
  elements.ntripConnect.disabled = !serviceRunning;
  elements.ntripConnect.textContent = ntrip.requested ? "修改/重连" : "连接基站";
  elements.ntripDisconnect.disabled = !ntrip.requested;
  if (ntrip.delivery_error || ntrip.error) elements.ntripHint.textContent = ntrip.delivery_error || ntrip.error;
  else if (ntrip.connected) elements.ntripHint.textContent = `${ntrip.host}:${ntrip.port}/${ntrip.mountpoint} · RTCM ${Number(ntrip.frames||0).toLocaleString()} 帧 · 已转发 ${Number(ntrip.forwarded_bytes||0).toLocaleString()} B`;
  else elements.ntripHint.textContent = ntrip.requested ? "正在连接并等待有效RTCM…" : "未连接；实时单点定位与采集不受影响";
}

function drawSkyPlot() {
  const canvas=elements.skyCanvas, rect=canvas.getBoundingClientRect(), ratio=window.devicePixelRatio||1;
  const width=Math.max(1,rect.width), height=Math.max(1,rect.height);
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
  }
  const ctx=canvas.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
  const cx=width/2,cy=height/2,radius=Math.max(60,Math.min(width,height)/2-43);
  ctx.strokeStyle="#344a63";ctx.lineWidth=1;
  [1,2/3,1/3].forEach(scale=>{ctx.beginPath();ctx.arc(cx,cy,radius*scale,0,Math.PI*2);ctx.stroke();});
  ctx.beginPath();ctx.moveTo(cx-radius,cy);ctx.lineTo(cx+radius,cy);ctx.moveTo(cx,cy-radius);ctx.lineTo(cx,cy+radius);ctx.stroke();
  ctx.fillStyle="#8ea4bd";ctx.font="12px ui-monospace, monospace";ctx.textAlign="center";ctx.textBaseline="middle";
  ctx.fillText("N",cx,cy-radius-17);ctx.fillText("E",cx+radius+17,cy);ctx.fillText("S",cx,cy+radius+17);ctx.fillText("W",cx-radius-17,cy);
  ctx.textAlign="left";ctx.fillText("0°",cx+radius+5,cy-12);ctx.fillText("30°",cx+radius*2/3+5,cy-12);ctx.fillText("60°",cx+radius/3+5,cy-12);
  if(!skySatellites.length){ctx.textAlign="center";ctx.fillStyle="#8ea4bd";ctx.fillText("等待完整 NAV-SAT 历元…",cx,cy);return;}
  skySatellites.filter(sat=>sat.elev_deg>=0).forEach(sat=>{
    const style=constellationStyle[sat.gnss_id]||{prefix:"U",color:"#d5deea"};
    const az=sat.azim_deg*Math.PI/180, radial=radius*(90-Math.min(90,sat.elev_deg))/90;
    const x=cx+radial*Math.sin(az),y=cy-radial*Math.cos(az),dot=Math.max(4,Math.min(8,3+sat.cno_dbhz/15));
    ctx.strokeStyle=style.color;ctx.fillStyle=style.color;ctx.lineWidth=sat.used?1.5:1.8;ctx.globalAlpha=sat.used?1:.72;
    ctx.beginPath();ctx.arc(x,y,dot,0,Math.PI*2);sat.used?ctx.fill():ctx.stroke();
    ctx.globalAlpha=1;ctx.fillStyle="#dcecff";ctx.font="11px ui-monospace, monospace";ctx.textAlign="left";ctx.fillText(`${style.prefix}${sat.sv_id}`,x+dot+3,y);
  });
}

function updateSky(satellites, age) {
  skySatellites=Array.isArray(satellites)?satellites:[];
  const visible=skySatellites.filter(sat=>sat.elev_deg>=0),used=visible.filter(sat=>sat.used).length;
  elements.skyCount.textContent=visible.length?`${used}/${visible.length} 使用`:`-- 颗`;
  elements.skyAge.textContent=Number.isFinite(age)?`${age.toFixed(1)} s 前更新`:"等待卫星历元";
  drawSkyPlot();
}

function validImu(value) {
  return value && ["sample", "timer_us", "ax_m_s2", "ay_m_s2", "az_m_s2",
    "temp_deg_c", "gx_deg_s", "gy_deg_s", "gz_deg_s"]
    .every(name => Number.isFinite(value[name]));
}

function chartNumber(value, span) {
  const magnitude = Math.abs(span);
  const digits = magnitude < .02 ? 4 : magnitude < .2 ? 3 : magnitude < 2 ? 2 : 1;
  return value.toFixed(digits);
}

function drawTimeChart(canvas, lines, unit) {
  const rect=canvas.getBoundingClientRect(), ratio=window.devicePixelRatio||1;
  const width=Math.max(1,rect.width), height=Math.max(1,rect.height);
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){
    canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);
  }
  const ctx=canvas.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
  const left=58,right=13,top=12,bottom=27,plotW=Math.max(1,width-left-right),plotH=Math.max(1,height-top-bottom);
  ctx.strokeStyle="rgba(104,142,178,.17)";ctx.fillStyle="#8ea4bd";ctx.lineWidth=1;
  ctx.font="10px ui-monospace, monospace";ctx.textBaseline="middle";
  if(!imuHistory.length){ctx.textAlign="center";ctx.fillText("等待IMU样本…",width/2,height/2);return;}
  const values=[];for(const item of imuHistory)for(const line of lines)values.push(item[line.key]);
  let minY=Math.min(...values),maxY=Math.max(...values),span=maxY-minY;
  if(!Number.isFinite(span)||span<1e-12){span=Math.max(Math.abs(maxY)*.1,.02);minY-=span/2;maxY+=span/2;}
  else{const margin=span*.12;minY-=margin;maxY+=margin;span=maxY-minY;}
  for(let i=0;i<=4;i++){
    const y=top+plotH*i/4,value=maxY-span*i/4;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();
    ctx.textAlign="right";ctx.fillText(chartNumber(value,span),left-7,y);
  }
  for(let i=0;i<=4;i++){
    const x=left+plotW*i/4;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,top+plotH);ctx.stroke();
  }
  const end=imuHistory[imuHistory.length-1].t;
  const start=Math.min(imuHistory[0].t,end-1000);
  const projectX=t=>left+(t-start)/Math.max(1,end-start)*plotW;
  const projectY=v=>top+(maxY-v)/span*plotH;
  for(const line of lines){
    ctx.strokeStyle=line.color;ctx.lineWidth=1.6;ctx.beginPath();
    imuHistory.forEach((item,index)=>{const x=projectX(item.t),y=projectY(item[line.key]);index?ctx.lineTo(x,y):ctx.moveTo(x,y);});
    ctx.stroke();
  }
  const shown=Math.round((end-start)/1000);
  ctx.fillStyle="#8ea4bd";ctx.textBaseline="alphabetic";ctx.textAlign="left";
  ctx.fillText(`-${shown}s`,left,height-7);ctx.textAlign="right";ctx.fillText("现在",width-right,height-7);
  ctx.textAlign="left";ctx.fillText(unit,left+5,top+11);
}

function drawImuCharts() {
  drawTimeChart(elements.accelCanvas,[
    {key:"ax",color:axisColors.x},{key:"ay",color:axisColors.y},{key:"az",color:axisColors.z}],"m/s²");
  drawTimeChart(elements.gyroCanvas,[
    {key:"gx",color:axisColors.x},{key:"gy",color:axisColors.y},{key:"gz",color:axisColors.z}],"deg/s");
  drawTimeChart(elements.temperatureCanvas,[{key:"temp",color:axisColors.temp}],"°C");
}

function updateImu(payload) {
  const imu=payload.imu, online=Boolean(payload.imu_online);
  elements.imuState.className=online?"online":"offline";
  elements.imuState.textContent=online?"IMU通信畅通":payload.service_running?"IMU数据中断":"采集服务已停止";
  elements.imuAge.textContent=Number.isFinite(payload.imu_age_s)
    ? `${payload.imu_age_s.toFixed(1)} s前 · 样本 ${imu?.sample ?? "--"}`:"等待首个IMU样本";
  if(!validImu(imu)){drawImuCharts();return;}
  elements.imuAx.textContent=fixed(imu.ax_m_s2,4);elements.imuAy.textContent=fixed(imu.ay_m_s2,4);
  elements.imuAz.textContent=fixed(imu.az_m_s2,4);elements.imuGx.textContent=fixed(imu.gx_deg_s,5);
  elements.imuGy.textContent=fixed(imu.gy_deg_s,5);elements.imuGz.textContent=fixed(imu.gz_deg_s,5);
  elements.imuTemp.textContent=fixed(imu.temp_deg_c,2);
  const key=`${imu.sample}:${imu.timer_us}`;
  if(key!==latestImuKey){
    latestImuKey=key;
    const timestamp=Number.isFinite(payload.imu_updated_unix_ms)?payload.imu_updated_unix_ms:Date.now();
    imuHistory.push({t:timestamp,ax:imu.ax_m_s2,ay:imu.ay_m_s2,az:imu.az_m_s2,
      gx:imu.gx_deg_s,gy:imu.gy_deg_s,gz:imu.gz_deg_s,temp:imu.temp_deg_c});
    const cutoff=timestamp-IMU_HISTORY_SECONDS*1000;
    imuHistory=imuHistory.filter(item=>item.t>=cutoff).slice(-(IMU_HISTORY_SECONDS+2));
  }
  drawImuCharts();
}

function updateValues(payload) {
  elements.positionState.className = `live-state ${payload.service_running ? "online" : "offline"}`;
  elements.positionState.lastElementChild.textContent = payload.service_running ? "定位服务：运行中" : "定位服务：已停止";
  elements.recordingState.className = `live-state ${payload.recording ? "online" : "idle"}`;
  elements.recordingState.lastElementChild.textContent = payload.recording ? "数据采集：保存中" : "数据采集：未保存";
  elements.startRecording.disabled = !payload.service_running || payload.recording;
  elements.stopRecording.disabled = !payload.service_running || !payload.recording;
  const ubxBytes = Number(payload.ubx?.recorded_bytes || 0);
  elements.recordHint.textContent = payload.recording
    ? `正在保存：${payload.session || "正在创建目录"}；UBX ${(ubxBytes / 1048576).toFixed(2)} MiB`
    : "实时位置继续更新；当前不写入CSV或UBX";
  elements.age.textContent = Number.isFinite(payload.age_s) ? `${payload.age_s.toFixed(1)} s前` : "--";
  elements.session.textContent = `采集会话：${payload.session || "--"}`;
  updateNtrip(payload.ntrip, payload.service_running);
  updateSky(payload.satellites, payload.satellites_age_s);
  updateImu(payload);
  if (!payload.ok || !payload.data) { elements.message.textContent = payload.message || "等待GNSS数据"; return; }
  const d = payload.data;
  elements.fix.textContent = d.fix_text || "--"; elements.satellites.textContent = d.num_sv ?? "--";
  elements.pdop.textContent = fixed(d.pdop, 2); elements.speed.textContent = fixed(d.ground_speed_m_s, 3);
  elements.hAcc.textContent = fixed(d.h_acc_m, 3); elements.latitude.textContent = fixed(d.lat_deg, 9, "°");
  elements.longitude.textContent = fixed(d.lon_deg, 9, "°"); elements.height.textContent = fixed(d.height_m, 3, " m");
  elements.velN.textContent = fixed(d.vel_n_m_s, 3); elements.velE.textContent = fixed(d.vel_e_m_s, 3);
  elements.velD.textContent = fixed(d.vel_d_m_s, 3); elements.gpsTime.textContent = gpsLabel(d);
  elements.message.textContent = payload.message || (payload.online ? "实时定位正常" : "GNSS结果没有更新");
  syncPoints(payload, d);
}

async function poll() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    updateValues(await response.json());
  } catch (error) {
    elements.positionState.className = "live-state offline";
    elements.positionState.lastElementChild.textContent = "定位服务：无法连接";
    elements.recordingState.className = "live-state idle";
    elements.recordingState.lastElementChild.textContent = "数据采集：未知";
    elements.imuState.className="offline";elements.imuState.textContent="网页连接中断";
    elements.startRecording.disabled = true; elements.stopRecording.disabled = true; elements.ntripConnect.disabled = true;
    elements.message.textContent = String(error);
  }
}
async function postAction(path, body) {
  const options = {method: "POST", headers: {"X-GNSS-Dashboard": "1"}, cache: "no-store"};
  if (body) { options.headers["Content-Type"] = "application/json"; options.body = JSON.stringify(body); }
  const response = await fetch(path, options), payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  return payload;
}
async function setRecording(action) {
  elements.startRecording.disabled = true; elements.stopRecording.disabled = true;
  elements.recordHint.textContent = action === "start" ? "正在开始采集…" : "正在停止并刷新文件…";
  try { elements.recordHint.textContent = (await postAction(`/api/recording/${action}`)).message; setTimeout(poll, 300); }
  catch (error) { elements.recordHint.textContent = String(error); setTimeout(poll, 1000); }
}

function localCoordinates() {
  if (!points.length) return [];
  const origin = points[0], cosLat = Math.cos(origin.lat_deg * Math.PI / 180);
  return points.map(p => ({x:(p.lon_deg-origin.lon_deg)*111319.4908*cosLat,
    y:(p.lat_deg-origin.lat_deg)*110574.0, carrier:p.carr_soln}));
}
function drawLocalTrack() {
  if (!localMode) return;
  const canvas=elements.canvas, rect=canvas.getBoundingClientRect(), ratio=window.devicePixelRatio||1;
  const width=Math.max(1,rect.width), height=Math.max(1,rect.height);
  if (canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)) {
    canvas.width=Math.round(width*ratio); canvas.height=Math.round(height*ratio);
  }
  const ctx=canvas.getContext("2d"); ctx.setTransform(ratio,0,0,ratio,0,0); ctx.clearRect(0,0,width,height);
  const local=localCoordinates(); if (!local.length) return;
  let minX=Math.min(...local.map(p=>p.x)), maxX=Math.max(...local.map(p=>p.x));
  let minY=Math.min(...local.map(p=>p.y)), maxY=Math.max(...local.map(p=>p.y));
  const range=Math.max(maxX-minX,maxY-minY,2);
  minX=(minX+maxX-range)/2; maxX=minX+range; minY=(minY+maxY-range)/2; maxY=minY+range;
  const pad=44, scale=Math.min((width-2*pad)/(maxX-minX),(height-2*pad)/(maxY-minY));
  const project=p=>[width/2+(p.x-(minX+maxX)/2)*scale,height/2-(p.y-(minY+maxY)/2)*scale];
  ctx.strokeStyle="rgba(104,142,178,.16)"; ctx.lineWidth=1;
  for(let i=1;i<10;i++){const x=width*i/10,y=height*i/10;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,height);ctx.stroke();ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(width,y);ctx.stroke();}
  local.forEach((p,index)=>{const [x,y]=project(p);ctx.fillStyle=pointColor(p);ctx.globalAlpha=index===local.length-1?1:.72;ctx.beginPath();ctx.arc(x,y,index===local.length-1?7:2.7,0,Math.PI*2);ctx.fill();});
  ctx.globalAlpha=1; ctx.fillStyle="#8ea4bd"; ctx.font="12px ui-monospace, monospace";
  ctx.fillText(`${range.toFixed(range<10?1:0)} m`,14,height-15);
}
function showLocal() {
  localMode=true; elements.canvas.hidden=false; elements.amap.hidden=true;
  $("localButton").classList.add("active"); $("amapButton").classList.remove("active"); drawLocalTrack();
}
function rebuildAmap() { clearAmapDots(); points.slice(-600).forEach(addAmapDot); updateAmapCurrent(); }
async function resolveAmapConfig() {
  try {
    const response=await fetch("/api/map-config",{cache:"no-store"}),value=await response.json();
    if(response.ok&&value.configured&&value.key) {
      return {key:value.key,security:value.securityJsCode||""};
    }
  } catch (_) { /* The normal status poll reports a Pi connection failure. */ }
  const localKey=localStorage.getItem("amapKey")||"";
  if(localKey) return {key:localKey,security:localStorage.getItem("amapSecurity")||""};
  return {key:"",security:""};
}
async function loadAmap() {
  const {key,security}=await resolveAmapConfig();
  if(!key){
    elements.message.textContent="这台手机尚未配置高德地图，请填写Key和安全密钥";
    $("settingsDialog").showModal();return;
  }
  localMode=false;elements.canvas.hidden=true;elements.amap.hidden=false;$("localButton").classList.remove("active");$("amapButton").classList.add("active");
  if(amapInstance){rebuildAmap();return;} if(amapLoading)return; amapLoading=true;
  window._AMapSecurityConfig={securityJsCode:security}; const script=document.createElement("script");
  script.src=`https://webapi.amap.com/maps?v=2.0&key=${encodeURIComponent(key)}`;
  const timer=setTimeout(()=>{if(!amapLoading)return;amapLoading=false;showLocal();elements.message.textContent="高德地图加载超时：手机热点当前没有可用外网";},10000);
  script.onload=()=>{clearTimeout(timer);try{amapLoading=false;amapInstance=new AMap.Map("amap",{zoom:17,viewMode:"2D"});rebuildAmap();}catch(error){showLocal();elements.message.textContent=`高德地图初始化失败：${error}`;}};
  script.onerror=()=>{clearTimeout(timer);amapLoading=false;showLocal();elements.message.textContent="高德地图资源加载失败：请检查热点外网、Key及安全密钥";};document.head.appendChild(script);
}

function openNtripDialog() {
  $("ntripHost").value=localStorage.getItem("ntripHost")||"ntrip.gnsswhu.cn";
  $("ntripPort").value=localStorage.getItem("ntripPort")||"2101";
  $("ntripMount").value=localStorage.getItem("ntripMount")||"WUH200CHN0";
  $("ntripUser").value=localStorage.getItem("ntripUser")||""; $("ntripPassword").value=""; $("ntripDialog").showModal();
}
async function connectNtrip(event) {
  event.preventDefault();
  const config={host:$("ntripHost").value.trim(),port:Number($("ntripPort").value),mountpoint:$("ntripMount").value.trim(),username:$("ntripUser").value,password:$("ntripPassword").value};
  elements.ntripHint.textContent="正在提交基站配置…";
  try {
    const result=await postAction("/api/ntrip/connect",config);
    localStorage.setItem("ntripHost",config.host);localStorage.setItem("ntripPort",String(config.port));localStorage.setItem("ntripMount",config.mountpoint);localStorage.setItem("ntripUser",config.username);
    $("ntripPassword").value="";$("ntripDialog").close();elements.ntripHint.textContent=result.message;setTimeout(poll,300);
  } catch(error){elements.ntripHint.textContent=String(error);}
}
async function disconnectNtrip() {
  elements.ntripDisconnect.disabled=true;
  try{elements.ntripHint.textContent=(await postAction("/api/ntrip/disconnect")).message;}catch(error){elements.ntripHint.textContent=String(error);}
  setTimeout(poll,300);
}

function appendCommandOutput(command, output) {
  const timestamp = new Date().toLocaleTimeString([], {hour12:false});
  const block = `[${timestamp}] pi$ ${command}\n${output || "（无输出）"}`;
  const current = elements.commandOutput.textContent.trim();
  elements.commandOutput.textContent = (current ? `${current}\n\n${block}` : block).slice(-32768);
  elements.commandOutput.scrollTop = elements.commandOutput.scrollHeight;
}

async function runConsoleCommand(command) {
  const value = String(command || "").trim();
  if (!value) return;
  elements.commandInput.value = "";
  if (value.toLowerCase() === "clear") {
    elements.commandOutput.textContent = "";
    return;
  }
  if (!commandHistory.length || commandHistory[commandHistory.length - 1] !== value) {
    commandHistory.push(value);
    if (commandHistory.length > 30) commandHistory.shift();
  }
  commandHistoryIndex = commandHistory.length;
  elements.commandRun.disabled = true;
  try {
    const response = await fetch("/api/command", {
      method: "POST", headers: {"X-GNSS-Dashboard":"1", "Content-Type":"application/json"},
      cache: "no-store", body: JSON.stringify({command:value})
    });
    const payload = await response.json();
    appendCommandOutput(value, payload.output || payload.message || `HTTP ${response.status}`);
    if (["status", "record start", "record stop", "base reconnect", "base disconnect"].includes(value.toLowerCase())) {
      setTimeout(poll, 300);
    }
  } catch (error) {
    appendCommandOutput(value, `请求失败：${error}`);
  } finally {
    elements.commandRun.disabled = false;
    elements.commandInput.focus();
  }
}

$("localButton").addEventListener("click",showLocal); $("amapButton").addEventListener("click",loadAmap);
$("fitButton").addEventListener("click",()=>{if(localMode)drawLocalTrack();else if(amapInstance)amapInstance.setFitView([...amapDots.map(item=>item.overlay),amapMarker].filter(Boolean));});
$("settingsButton").addEventListener("click",()=>{$("amapKey").value=localStorage.getItem("amapKey")||"";$("amapSecurity").value=localStorage.getItem("amapSecurity")||"";$("settingsDialog").showModal();});
$("saveMapSettings").addEventListener("click",event=>{event.preventDefault();localStorage.setItem("amapKey",$("amapKey").value.trim());localStorage.setItem("amapSecurity",$("amapSecurity").value.trim());$("settingsDialog").close();if(amapInstance)location.reload();else loadAmap();});
elements.startRecording.addEventListener("click",()=>setRecording("start"));elements.stopRecording.addEventListener("click",()=>setRecording("stop"));
elements.ntripConnect.addEventListener("click",openNtripDialog);elements.ntripDisconnect.addEventListener("click",disconnectNtrip);$("saveNtrip").addEventListener("click",connectNtrip);
elements.commandForm.addEventListener("submit",event=>{event.preventDefault();runConsoleCommand(elements.commandInput.value);});
document.querySelectorAll("[data-command]").forEach(button=>button.addEventListener("click",()=>runConsoleCommand(button.dataset.command)));
elements.commandInput.addEventListener("keydown",event=>{
  if (!commandHistory.length || !["ArrowUp","ArrowDown"].includes(event.key)) return;
  event.preventDefault();
  commandHistoryIndex = event.key === "ArrowUp"
    ? Math.max(0, commandHistoryIndex - 1)
    : Math.min(commandHistory.length, commandHistoryIndex + 1);
  elements.commandInput.value = commandHistoryIndex < commandHistory.length ? commandHistory[commandHistoryIndex] : "";
});
window.addEventListener("resize",()=>{drawLocalTrack();drawSkyPlot();drawImuCharts();});
poll();setInterval(poll,1000);
// Diagnostic/deep link used to verify the complete online-map path without
// changing the default lightweight local-track view.
if(new URLSearchParams(location.search).get("map")==="1")setTimeout(loadAmap,100);
