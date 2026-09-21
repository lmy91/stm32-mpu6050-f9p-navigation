# 手机/PC实时定位网页

树莓派开机运行两个常驻服务：`capture_serial.py`独占UART、持续解码并发布实时
位置，`live_dashboard.py`提供局域网页面和保存开关。开机默认不写采集文件，
网页仍能实时显示位置。只有输入命令或点击“开始采集”后才创建数据目录并同步
保存三个CSV和`f9p.ubx`；点击“停止采集”只停止保存，实时定位不中断。网页
进程本身绝不打开`/dev/ttyAMA0`或`/dev/ttyAMA2`。

## 页面功能

- 1 Hz当前位置、经纬度和高程；未保存时地图只显示一个当前点，保存期间才按
  当前会话累计离散轨迹点，点之间不连线；
- 3D、RTK浮点、RTK固定状态；
- 卫星数、PDOP、水平精度和数据新鲜度；
- NED速度和地速；
- IMU实时监测：串口继续按100 Hz完整采集，网页每秒抽取一个最新样本，显示
  三轴加速度（m/s²）、三轴角速度（deg/s）、温度（°C）及最近120秒时间曲线；
- 地图下方1 Hz卫星天空图：按方位角/仰角绘制GPS、Galileo、北斗、
  GLONASS、QZSS和SBAS，实心表示用于导航，空心表示未使用；
- 无需Key的WGS-84本地等比例轨迹；
- 可选高德在线地图，显示时WGS-84转换为GCJ-02，CSV保持原始WGS-84；
- 手机与电脑自适应布局。
- 分别显示“定位服务”和“数据采集”状态；
- 网页一键开始/停止三个CSV和原始`f9p.ubx`保存，并显示当前UBX文件大小。
- 网页填写NTRIP服务器、挂载点和账号后，一键连接/断开差分基站；RTCM由采集
  进程经同一UART和STM32信用窗口转发到F9P，不会产生串口争用。
- 页面底部提供受限服务控制台，包含命令输入和结果窗口，可查询服务、日志、
  网络与磁盘，也可控制保存和NTRIP；支持方向键调出最近输入的命令。

高德Key可以保存在每台手机浏览器的`localStorage`中，也可以统一放在树莓派
`/home/lmy/.config/gnss-imu/amap.json`中。后者不会写入CSV或提交到Git，并可让
新手机首次打开页面时直接加载地图。该文件格式为：

```json
{"key":"高德Web JS API Key","securityJsCode":"高德安全密钥"}
```

建议执行`chmod 600 /home/lmy/.config/gnss-imu/amap.json`。Web JS API工作原理
决定了这两个值最终会送到浏览器，因此网页服务只应开放在可信热点/局域网。
在线高德底图仍需要树莓派具备外网；没有外网时，本地WGS-84等比例轨迹正常。


IMU网页曲线复用采集进程已经解码的数据，不会新增STM32命令或串口输出。每个
浏览器客户端只增加约1次/秒的局域网状态请求，因此不会影响100 Hz串口采集或
CSV/UBX记录；浏览器刷新后曲线从空白重新累计。

## 安装为开机服务

在树莓派项目目录执行：

```bash
sudo cp raspberry_pi5/systemd/gnss-imu-dashboard.service \
  /etc/systemd/system/gnss-imu-dashboard.service
sudo cp raspberry_pi5/systemd/gnss-imu-logger.service \
  /etc/systemd/system/gnss-imu-logger.service
sudo cp raspberry_pi5/systemd/gnss-imu-time-sync.service \
  /etc/systemd/system/gnss-imu-time-sync.service
chmod +x raspberry_pi5/base_station_ctl.py
sudo ln -sf \
  /home/lmy/stm32-mpu6050-f9p-navigation/raspberry_pi5/base_station_ctl.py \
  /usr/local/bin/gnss-imu-base
sudo systemctl daemon-reload
sudo systemctl enable --now gnss-imu-logger.service
sudo systemctl enable --now gnss-imu-dashboard.service
sudo systemctl enable gnss-imu-time-sync.service
```

查看状态和日志：

```bash
systemctl status gnss-imu-dashboard.service
journalctl -u gnss-imu-dashboard.service -f
```

两个服务异常退出都会自动恢复。`gnss-imu-time-sync.service`是`oneshot`校时服务，
开机后等待`/run/gnss-imu/live.json`出现有效F9P时间，一次性校正系统时钟后退出；
离线冷启动（无网络NTP）时保证采集文件时间戳正确。`/run/gnss-imu/recording.enabled`是易失的保存
开关，因此重新开机默认只启动实时定位，不会自动继续上一次保存。
`/run/gnss-imu/ntrip.json`同样是易失文件，基站连接和密码不会跨重启保留，
也不会写入项目或采集目录。

命令行控制保存：

```bash
gnss-imu-record-start
gnss-imu-record-stop
```

命令行控制基站（密码采用隐藏输入，不会写入终端历史）：

```bash
# 交互输入服务器、挂载点、用户名和密码并等待连接结果
gnss-imu-base connect

# 查看NTRIP、RTCM帧和STM32转发状态
gnss-imu-base status

# 使用本次开机已保存于/run内的配置强制重连
gnss-imu-base reconnect

# 断开基站并立即清除/run内的账号
gnss-imu-base disconnect
```

终端命令和网页“连接基站”操作同一个`/run/gnss-imu/ntrip.json`，两者状态完全
一致。不要把密码直接写入命令；脚本也不提供明文密码参数。

## 网页服务控制台

页面底部可以输入下列命令，也可以点击常用命令按钮：

```text
help
status
service status logger
service status dashboard
service restart logger
service restart dashboard
record start
record stop
base status
base reconnect
base disconnect
logs logger 40
logs dashboard 40
network
disk
clear
```

控制台只解析上述白名单，不调用Shell，因此不能执行`sudo`、任意程序、关机、
删除或读取密码。`clear`仅清空当前浏览器的结果显示。网页仍未配置账号认证，
只应在可信的树莓派热点或局域网内使用。

重启采集服务会产生约3秒的实时数据间断，因此保存期间会拒绝执行，必须先
`record stop`。重启网页服务不影响串口采集，页面约4秒后恢复。两个操作均由
同一用户向已知进程发送正常退出信号，不开放root命令执行。

每次由停止切换到开始都会创建一个新的`YYYYMMDDHHMMSS`会话目录。停止时立即
刷新并关闭三个CSV和`f9p.ubx`。正常关机也会安全关闭文件；“直接关机”应使用系统关机命令
或树莓派电源键触发的正常关机，不能在写入过程中直接拔电源。

## 浏览器访问

当前电脑与树莓派网线直连配置：

```text
http://192.168.137.2:8080
```

手机需要和树莓派之间存在可达的局域网路径。最简单的方法是让手机与树莓派
连接同一个Wi-Fi或手机热点，然后查询树莓派无线地址：

```bash
hostname -I
```

在手机浏览器中访问`http://<树莓派无线IP>:8080`。如果手机只与Windows处于
同一上游Wi-Fi，它通常无法直接穿过Windows ICS访问网线后的
`192.168.137.2`；此时让树莓派同时接入该Wi-Fi/手机热点，或在Windows上另行
设置端口转发。

## 接口

网页使用两个只读接口和两个同源POST控制接口：

```text
GET /api/status          最新GNSS位置、速度、精度和链路新鲜度
GET /api/track?limit=600 当前会话最近600个轨迹点，最大5000点
GET /api/map-config       树莓派本地高德Web JS API配置
POST /api/recording/start 开始新采集会话
POST /api/recording/stop  刷新并停止保存
POST /api/ntrip/connect    写入易失配置并连接NTRIP基站
POST /api/ntrip/disconnect 断开基站并清除易失账号
POST /api/command          执行受限服务控制台白名单命令
```

服务默认监听所有局域网接口的TCP 8080端口，没有公网身份验证。只应在可信
局域网使用，不要把该端口直接映射到公网。后续组合导航模块可以保持相同接口，
再增加100 Hz组合位置、速度和姿态数据源。

## 网页连接按钮报“只读文件系统”

旧服务若只更新程序、没有重新复制并重启systemd单元，网页可能无法写入新增的
基站控制文件。执行：

```bash
cd /home/lmy/stm32-mpu6050-f9p-navigation
sudo cp raspberry_pi5/systemd/gnss-imu-dashboard.service \
  /etc/systemd/system/gnss-imu-dashboard.service
sudo systemctl daemon-reload
sudo systemctl restart gnss-imu-dashboard.service
```

新版服务让采集器和网页共同持有`RuntimeDirectory=gnss-imu`，单独重启任一服务
也不会使另一个服务保留失效的只读挂载。
