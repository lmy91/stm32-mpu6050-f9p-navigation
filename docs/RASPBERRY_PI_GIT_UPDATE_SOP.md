# 树莓派生产软件 Git 更新 SOP

本文记录从 GitHub 更新树莓派 5 正式采集软件的标准操作流程。正式部署仓库为：

<https://github.com/lmy91/pi5-mpu6050-f9p-logger>

STM32 固件和串口协议仍以父项目 `stm32-mpu6050-f9p-navigation` 为上游；本文只处理
树莓派端软件，不烧写 STM32 固件。

## 1. 适用环境

- Windows 电脑通过网线连接树莓派；
- Windows 以太网地址通常为 `192.168.137.1`；
- 树莓派地址为 `192.168.137.2`；
- SSH 快捷主机名为 `pi5-ics`，用户为 `lmy`；
- 树莓派项目目录为 `/home/lmy/pi5-mpu6050-f9p-logger`；
- 正式更新分支为远程 `origin/main`。

若现场用户名、地址或项目路径不同，先替换下文对应值。不要把本父项目中的
`raspberry_pi5/` 历史代码复制到生产目录。

## 2. 更新前检查

在 Windows PowerShell 中确认树莓派可达：

```powershell
Test-NetConnection 192.168.137.2 -Port 22
ssh pi5-ics
```

登录树莓派后执行：

```bash
cd /home/lmy/pi5-mpu6050-f9p-logger

# 记录更新前版本，供失败时回滚
OLD_HEAD=$(git rev-parse HEAD)
echo "更新前版本：$OLD_HEAD"

# 获取远程最新分支和标签
git fetch origin --prune --tags

# 检查本地状态、远程最新版本和落后提交
git status --short --branch
git log -1 --oneline HEAD
git log -1 --oneline origin/main
git rev-list --left-right --count HEAD...origin/main

# 检查是否正在保存数据
if [ -e /run/gnss-imu/recording.enabled ]; then
    echo "当前正在保存数据"
else
    echo "当前未保存数据"
fi
```

如果正在保存，必须先安全停止：

```bash
gnss-imu-record-stop
```

确认当前会话文件已经停止增长后再继续。禁止更新时直接断电。

## 3. 保留内容与覆盖范围

更新时保留以下现场内容：

- `/home/lmy/pi5-mpu6050-f9p-logger/data/decoded/` 中的历史采集数据；
- `/home/lmy/.config/gnss-imu/amap.json` 中的地图配置；
- 运行时由网页或终端输入的 NTRIP 信息不应写入 Git 仓库。

正式树莓派不应直接修改仓库代码。以下覆盖更新会丢弃项目目录内已跟踪文件的
本地改动和权限变化。如果确有需要保留的现场代码修改，先导出补丁并审查：

```bash
git diff > "$HOME/pi5-local-before-update.patch"
```

禁止使用 `git clean -fdx`，因为它会把被 Git 忽略的历史采集数据一并删除。

## 4. 停止服务并覆盖到最新版

```bash
cd /home/lmy/pi5-mpu6050-f9p-logger

sudo systemctl stop \
  gnss-imu-dashboard.service \
  gnss-imu-time-sync.service \
  gnss-imu-logger.service

# 用远程 main 完整覆盖旧代码
git reset --hard origin/main

# 安装脚本需要的执行权限
chmod +x scripts/*.sh \
  raspberry_pi5/*.sh \
  raspberry_pi5/base_station_ctl.py \
  raspberry_pi5/gnss_time_sync.py
```

`git reset --hard origin/main` 是生产机标准覆盖步骤，只能在已经确认不需要保留
本地代码改动后执行。它不会删除未跟踪或被忽略的 `data/decoded/` 数据。

## 5. 更新后测试与服务重装

先运行全部树莓派端自动测试：

```bash
python3 -m unittest \
  raspberry_pi5.test_gnss_time_sync \
  raspberry_pi5.test_ntrip_client \
  raspberry_pi5.test_live_dashboard \
  tools.test_capture_serial
```

预期结果为所有测试通过并显示 `OK`。测试通过后重新生成并安装 systemd 服务：

```bash
./scripts/install_services.sh
```

该脚本会按当前用户名和项目绝对路径重新生成三个服务、命令软链接并启动服务。

如果测试失败，不要启动新版本，直接执行本文“失败回滚”步骤。

## 6. 清理无用残留

只清理可再生成的 Python 缓存，不删除采集数据和设备配置：

```bash
find raspberry_pi5 tools -type f -name '*.pyc' -delete
find raspberry_pi5 tools -type f -name '*.pyo' -delete
find raspberry_pi5 tools -depth -type d -name __pycache__ -empty -delete
find raspberry_pi5 tools -depth -type d -name .pytest_cache -empty -delete
```

检查家目录里是否存在旧项目副本：

```bash
find /home/lmy -maxdepth 3 -type d \
  \( -iname '*pi5*mpu*' -o -iname '*gnss*imu*' -o -iname '*f9p*logger*' \) \
  -print
```

正常情况下只应有正式项目目录和 `~/.config/gnss-imu` 配置目录。删除旧副本前，
必须逐个核对其中是否包含未归档的 CSV、UBX、密钥或现场配置；不得批量删除搜索
结果，也不得删除 `~/.config/gnss-imu`。

安装脚本会给个别 Python 文件增加执行权限。若仅希望 `git status` 忽略这种权限
差异，可在该树莓派仓库中执行：

```bash
git config core.fileMode false
```

## 7. 部署验收

在树莓派上执行：

```bash
cd /home/lmy/pi5-mpu6050-f9p-logger

# 必须与远程 main 完全一致
git rev-parse HEAD
git rev-parse origin/main
git status --short --branch

# 三个服务都应为 enabled 和 active
systemctl is-enabled \
  gnss-imu-logger.service \
  gnss-imu-dashboard.service \
  gnss-imu-time-sync.service
systemctl is-active \
  gnss-imu-logger.service \
  gnss-imu-dashboard.service \
  gnss-imu-time-sync.service

# 完整设备自检
gnss-imu-diagnose

# 检查本次启动后的告警
journalctl \
  -u gnss-imu-logger.service \
  -u gnss-imu-dashboard.service \
  -u gnss-imu-time-sync.service \
  -p warning --since '10 minutes ago' --no-pager
```

验收判据：

- `HEAD` 与 `origin/main` 哈希相同；
- 三个服务均已启用且处于 `active`；
- IMU 约为 100 Hz，GNSS、RAWX 和 SAT 计数持续增加；
- `lost` 不持续增加，UART 没有被其他进程争用；
- `vcgencmd get_throttled` 为 `0x0`；
- 日志中没有新的 warning/error；
- 默认 `REC=OFF`，只有明确开始采集后才保存文件。

回到 Windows PowerShell 验证网页：

```powershell
Invoke-WebRequest -UseBasicParsing http://192.168.137.2:8080/ -TimeoutSec 8 |
  Select-Object StatusCode, RawContentLength
```

预期 HTTP 状态码为 `200`。也可直接在浏览器打开
<http://192.168.137.2:8080>。

## 8. 失败回滚

如果测试、服务或现场数据流验收失败，在同一个 SSH 会话中执行：

```bash
cd /home/lmy/pi5-mpu6050-f9p-logger

sudo systemctl stop \
  gnss-imu-dashboard.service \
  gnss-imu-time-sync.service \
  gnss-imu-logger.service

git reset --hard "$OLD_HEAD"
./scripts/install_services.sh
gnss-imu-diagnose
```

如果 SSH 已经断开，`OLD_HEAD` 变量会丢失。可用以下命令查找更新前提交，再把
选定哈希代入回滚命令：

```bash
git reflog --date=iso -n 10
git reset --hard <确认无误的更新前提交哈希>
./scripts/install_services.sh
```

回滚后保留失败版本的提交哈希、测试输出和三个服务的日志，供后续定位；不要通过
反复重启掩盖串口争用、依赖缺失或协议不兼容问题。

## 9. 更新记录模板

每次更新后在维护记录中填写：

```text
更新时间：
操作人员：
更新前提交：
更新后提交：
测试结果：
三个服务状态：
IMU/GNSS/RAWX 实时状态：
网页 HTTP 验证：
是否保留现场补丁：
是否发现并清理旧副本：
异常及处理：
```
