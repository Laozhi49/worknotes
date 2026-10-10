Windows 传文件到 Linux 板子（比如你现在的 **RK3576 LubanCat / Buildroot / Ubuntu rootfs**）常用有几种方式，取决于板子有没有网络和 SSH。

---

## 方法1：scp（最推荐，Linux板子有 ssh/dropbear）

### 1. 确认板子 IP

板子执行：

```bash
ip addr
```

例如：

```
wlan0:
    inet 192.168.1.100
```

假设 IP 是：

```
192.168.1.100
```

---

### 2. Windows 使用 scp

Windows 10/11 自带 OpenSSH，可以直接在 PowerShell：

```powershell
scp 文件路径 root@192.168.1.100:/目标目录
```

例如：

把 Windows：

```
D:\test.sh
```

传到板子：

```
/data/
```

执行：

```powershell
scp D:\test.sh root@192.168.1.100:/data/
```

输入 Linux 密码即可。

一般可以使用让 scp 使用传统 SCP 协议 -O 参数
```powershell
scp -O D:\test.sh root@192.168.1.100:/data/
```

---

### 3. 传整个目录

例如：

```
D:\robot_ws
```

传到：

```
/data/
```

```powershell
scp -r D:\robot_ws root@192.168.1.100:/data/
```
或
```powershell
scp -O -r D:\robot_ws root@192.168.1.100:/data/
```
---

## 方法2：WinSCP（图形界面，最方便）

如果你经常传 ROS 工作空间、脚本，推荐这个。

下载：

[https://winscp.net/](https://winscp.net/)

连接配置：

```
文件协议:
    SCP 或 SFTP

主机名:
    192.168.1.100

用户名:
    root

密码:
    板子密码
```

登录后：

左边 Windows 文件

右边 Linux 文件

直接拖拽。

你的场景：

```
Windows
   |
   |
 WinSCP
   |
   |
RK3576板子

/data/workspace/
```

非常方便。

---

## 方法3：USB盘/U盘

如果板子没有网络：

Windows：

```
复制文件到U盘
```

Linux：

插入后：

```bash
ls /dev/sd*
```

挂载：

```bash
mount /dev/sda1 /mnt
```

复制：

```bash
cp /mnt/test.zip /data/
```

---

## 方法4：adb（如果是开发板）

有些 Rockchip 板子支持：

Windows：

```cmd
adb devices
```

传：

```cmd
adb push test.sh /data/
```

不过 Buildroot 默认一般没有 adb。

---

## 方法5：串口传输（不推荐）

你现在用：

```
/dev/ttyS10
```

这个是机器人 MCU 通信串口。

理论可以 xmodem/zmodem：

```
sz 文件
rz
```

但是速度慢，而且需要工具。

---

## 针对你现在 RK3576 + Buildroot + dropbear 环境

你之前提到：

```
dropbear
wpa_supplicant
/data目录可写
root登录
```

所以最适合：

### Windows → 板子

```powershell
scp xxx root@板子IP:/data/
```

或者直接：

```
WinSCP
```

---

如果你不知道板子的 IP，可以在板子：

```bash
ip -4 addr
```

查看，例如：

```
192.168.1.56
```

然后：

```powershell
scp robot_base_manager_launch.sh root@192.168.1.56:/data/
```

即可。你现在这个环境应该已经可以直接用了。
