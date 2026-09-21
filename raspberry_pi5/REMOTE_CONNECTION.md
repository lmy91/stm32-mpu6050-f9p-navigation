# 电脑与树莓派5直连远程连接配置

本文记录2026-09-11实机验证通过的Windows电脑与树莓派5直连方案。电脑通过
Wi-Fi访问互联网，并通过一根网线连接树莓派；Windows Internet Connection
Sharing（ICS）同时向树莓派提供本地连接和互联网出口。

## 1. 已验证网络参数

```text
互联网 / WUH2 NTRIP
        ↓
Windows WLAN
        ↓ Internet Connection Sharing
Windows 以太网：192.168.137.1/24
        ↕ 直连网线
树莓派 eth0：192.168.137.2/24
树莓派网关、DNS：192.168.137.1
```

树莓派同时保留旧的`192.168.50.2/24`地址，以便电脑恢复为
`192.168.50.1/24`时仍可使用原来的直连配置。

## 2. Windows开启网络共享

1. 打开“控制面板 → 网络和 Internet → 网络连接”。
2. 右键当前可以上网的`WLAN`，选择“属性 → 共享”。
3. 勾选“允许其他网络用户通过此计算机的 Internet 连接来连接”。
4. 家庭网络连接选择`以太网`并确定。

启用后，Windows通常会自动把以太网设为`192.168.137.1/24`。可在
PowerShell中检查：

```powershell
Get-NetIPAddress -InterfaceAlias "以太网" -AddressFamily IPv4
Get-Service SharedAccess
```

预期以太网地址为`192.168.137.1`，`SharedAccess`服务处于`Running`状态。

## 3. 树莓派有线连接配置

本机NetworkManager连接名称为`pc-direct`。从其他可用链路登录树莓派后执行：

```bash
sudo nmcli connection modify pc-direct \
  ipv4.method manual \
  ipv4.addresses "192.168.50.2/24,192.168.137.2/24" \
  ipv4.gateway 192.168.137.1 \
  ipv4.dns 192.168.137.1 \
  ipv4.never-default no
sudo nmcli connection up pc-direct
```

若连接名称不同，先用以下命令查询，再替换`pc-direct`：

```bash
nmcli -t -f NAME,DEVICE,TYPE,STATE connection show
```

检查地址、路由和互联网：

```bash
ip -4 -brief address show eth0
ip -4 route
ping -c 2 192.168.137.1
ping -c 2 1.1.1.1
getent hosts ntrip.gnsswhu.cn
```

## 4. Windows SSH快捷连接

电脑使用用户名`lmy`及已有密钥：

```text
C:/Users/12597/.ssh/pi5nav_ed25519
```

在`C:/Users/12597/.ssh/config`中保留以下配置：

```sshconfig
Host pi5-ics
    HostName 192.168.137.2
    User lmy
    IdentityFile C:/Users/12597/.ssh/pi5nav_ed25519
    IdentitiesOnly yes
```

之后在PowerShell中直接登录：

```powershell
ssh pi5-ics
```

也可以显式登录：

```powershell
ssh -i C:\Users\12597\.ssh\pi5nav_ed25519 lmy@192.168.137.2
```

验证结果应显示树莓派主机名`lmy`。

## 5. IPv4配置错误时的应急登录

即使两端IPv4不在同一网段，只要物理链路和SSH服务正常，仍可通过IPv6链路
本地地址登录。先在Windows查找树莓派邻居：

```powershell
Get-NetNeighbor -InterfaceAlias "以太网" -AddressFamily IPv6 |
  Where-Object { $_.IPAddress -like "fe80::*" -and $_.State -ne "Unreachable" }
```

本次实测树莓派MAC为`88-A2-9E-3F-A4-F6`。找到对应的`fe80::`地址后，将
Windows以太网接口编号附在地址末尾，例如：

```powershell
ssh -6 -i C:\Users\12597\.ssh\pi5nav_ed25519 `
  "lmy@fe80::4fbd:3564:b8c6:ea33%19"
```

其中IPv6地址和`%19`接口编号必须以当前电脑查询结果为准，不能在其他电脑上
直接照抄。

## 6. GPIO串口远程配置

树莓派5必须关闭GPIO串口登录控制台，同时保留UART硬件。通过SSH执行：

```bash
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
```

树莓派5的`/dev/serial0 -> ttyAMA10`默认指向板载三针调试UART，不是40针排针
的GPIO14/15。必须另外在`/boot/firmware/config.txt`的`[all]`段加入：

```text
dtparam=uart0=on
dtoverlay=uart0-pi5
```

然后重启：

```bash
sudo reboot
```

执行后，`/boot/firmware/cmdline.txt`中不应再包含`console=serial0,115200`。
重启后检查：

```bash
ls -l /dev/ttyAMA0 /dev/serial0
pinctrl get 14,15
systemctl is-active serial-getty@ttyAMA0.service
```

本机实测40针GPIO14/15对应`/dev/ttyAMA0`；`/dev/serial0`仍指向调试接口
`ttyAMA10`，不得用于本项目。GPIO14应为`TXD0`、GPIO15应为`RXD0`，
`serial-getty@ttyAMA0`应为`inactive`。

配置460800 bit/s并临时查看STM32输出：

```bash
sudo stty -F /dev/ttyAMA0 460800 raw -echo -ixon -ixoff \
  -crtscts clocal cread cs8 -cstopb -parenb
sudo timeout 5 dd if=/dev/ttyAMA0 bs=256 status=none | od -An -tx1 -v
```

协议包含二进制数据，因此十六进制显示最可靠。采集程序运行时不要再执行上述
读取命令，否则两个进程会争用同一个串口。

## 7. 本次验证结果

- Windows到`192.168.137.2`往返延迟小于1 ms。
- `ssh pi5-ics`使用密钥可直接登录，无需密码。
- 树莓派可以访问`1.1.1.1`并解析`ntrip.gnsswhu.cn`。
- GPIO14/15已经复用为UART0，串口登录控制台已经关闭。
- `/dev/ttyAMA0`在2秒内实测收到28208字节STM32协议v3数据。
