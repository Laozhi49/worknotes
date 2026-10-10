# rootfs的组成

`rootfs`（root filesystem，根文件系统）是 Linux 系统启动后看到的 `/` 目录下的整个文件系统内容。它包含运行 Linux 所需要的用户空间环境，包括程序、配置文件、库、设备节点、脚本等。

简单理解：

> **Kernel（内核）负责管理硬件，rootfs 提供运行环境。**

典型嵌入式 Linux（比如你使用的 RK3576、NanoPi 等）中的 rootfs 大致包含以下部分：

```
rootfs/
├── bin/
├── sbin/
├── lib/
├── usr/
├── etc/
├── dev/
├── proc/
├── sys/
├── run/
├── tmp/
├── var/
├── home/
├── root/
├── mnt/
├── opt/
└── boot/
```

---

## 1. /bin：基础用户命令

存放系统启动后最基本的可执行程序。

例如：

```
/bin/sh
/bin/ls
/bin/cp
/bin/mount
/bin/cat
```

很多嵌入式系统使用 BusyBox：

```
/bin/busybox
```

然后：

```
ls -> busybox
cp -> busybox
mount -> busybox
```

一个几 MB 的 BusyBox 就能提供几十到几百个 Linux 命令。

---

## 2. /sbin：系统管理程序

主要给管理员使用：

例如：

```
/sbin/init
/sbin/reboot
/sbin/ifconfig
/sbin/fsck
```

启动流程中常用。

---

## 3. /lib：动态库和内核模块

存放程序运行需要的共享库。

例如：

```
libc.so
libpthread.so
libm.so
ld-linux-aarch64.so.1
```

比如一个 ARM64 程序：

```
./robot_node
```

执行时需要：

```
robot_node
    |
    +-- libc.so
    |
    +-- libstdc++.so
    |
    +-- libgcc_s.so
```

这些通常在：

```
/lib
/usr/lib
```

里面。

还包括：

```
/lib/modules/
```

内核驱动模块：

```
xxx.ko
```

例如：

```
wifi.ko
camera.ko
can.ko
```

---

## 4. /etc：系统配置文件

这是 rootfs 最重要的目录之一。

包含：

### 网络

```
/etc/network/
/etc/wpa_supplicant.conf
```

WiFi 配置：

```
/etc/wpa_supplicant.conf
```

---

### 用户

```
/etc/passwd
/etc/group
/etc/shadow
```

---

### 启动配置

```
/etc/init.d/
/etc/systemd/
```

例如：

```
/etc/systemd/system/
    robot_base_monitor.service
```

你之前配置 ROS2 自启动，就是这里。

---

### 动态库配置

```
/etc/ld.so.conf
```

---

## 5. /usr：主要应用程序

通常 rootfs 最大的部分。

结构：

```
/usr
├── bin
├── lib
├── include
├── share
└── local
```

例如：

ROS2：

```
/usr/ros/
├── humble/
├── jazzy/
```

Python：

```
/usr/lib/python3.10/
```

GStreamer：

```
/usr/lib/gstreamer-1.0/
```

---

## 6. /dev：设备节点

Linux 把硬件抽象成文件。

例如：

串口：

```
/dev/ttyS0
/dev/ttyUSB0
```

摄像头：

```
/dev/video0
```

I2C：

```
/dev/i2c-0
```

SPI：

```
/dev/spidev0.0
```

GPIO：

```
/dev/gpiochip0
```

这些不是普通文件，而是由：

```
udev
mdev
devtmpfs
```

动态创建。

---

## 7. /proc：内核信息

虚拟文件系统。

例如：

查看 CPU：

```
cat /proc/cpuinfo
```

内存：

```
cat /proc/meminfo
```

进程：

```
ls /proc
```

例如：

```
/proc/1234/
```

表示 PID=1234 的进程。

---

## 8. /sys：硬件设备模型

Linux kernel sysfs。

例如：

查看设备：

```
/sys/class/
```

GPIO：

```
/sys/class/gpio/
```

网卡：

```
/sys/class/net/
```

---

## 9. /run：运行时数据

保存启动后的临时数据。

例如：

```
/run/systemd/
```

PID 文件：

```
/run/xxx.pid
```

---

## 10. /tmp：临时文件

重启后通常清空。

例如：

```
/tmp/cache
/tmp/socket
```

---

## 11. /var：可变数据

日志、缓存等。

例如：

```
/var/log/
```

系统日志：

```
/var/log/messages
```

apt：

```
/var/cache/
```

---

## 12. /home 和 /root

用户目录：

普通用户：

```
/home/user
```

管理员：

```
/root
```

---

## 13. /boot

启动相关文件。

嵌入式 ARM 中可能包含：

```
Image
dtb
extlinux.conf
boot.scr
```

例如 RK3576：

```
/boot/
├── Image
├── rk3576-xxx.dtb
└── extlinux/
```

不过很多 ARM 板子这些放在独立 boot 分区。

---

# 一个嵌入式 Linux 启动关系

以 RK3576 为例：

```
BootROM
   |
   v
SPL
   |
   v
U-Boot
   |
   +----加载 Kernel
   |
   +----加载 DTB
   |
   +----挂载 rootfs
              |
              v
          /sbin/init
              |
              v
          systemd
              |
              v
          ROS2节点
```

其中：

```
Kernel
```

提供：

* CPU管理
* 内存管理
* 驱动
* 网络协议栈

而：

```
rootfs
```

提供：

* bash
* ssh
* python
* ROS2
* 应用程序
* 配置文件

---

# rootfs 通常怎么生成？

常见方式：

### 1. Buildroot

生成：

```
output/images/rootfs.tar
```

里面就是 rootfs。

---

### 2. Yocto

生成：

```
core-image-minimal
```

---

### 3. Ubuntu/Debian

例如：

```
debootstrap
```

生成：

```
ubuntu-rootfs/
```

---

### 4. 厂商 SDK

例如 Rockchip：

```
rk3576_linux_sdk/
    build/
    ubuntu/
    rootfs/
```

里面可能直接提供：

```
rootfs.img
```

---

# rootfs 和 rootfs.img 的区别

| 名称         | 含义         |
| ---------- | ---------- |
| rootfs目录   | 解压后的文件     |
| rootfs.tar | 打包文件       |
| rootfs.img | 文件系统镜像     |
| squashfs   | 压缩只读rootfs |
| ext4.img   | 可读写rootfs  |

例如：

```
rootfs.ext4
```

烧录：

```
eMMC
 |
 +-- boot分区
 |
 +-- rootfs分区
```

挂载后：

```
/
├── bin
├── etc
├── usr
...
```

---

对于你之前使用的 **RK3576 + ROS2 + RKNN 环境**，rootfs 里面通常还会额外包含：

```
/opt/ros/
/usr/lib/aarch64-linux-gnu/
~/robot_ws/install/
/usr/bin/rknn_server
/usr/lib/librknnrt.so
/usr/lib/gstreamer-1.0/
```

这些都是应用运行环境的一部分。你如果做 RK3576 固件裁剪，实际上主要就是在裁剪 rootfs。

因此，判断一个嵌入式系统能否正常启动和运行应用，关键在于确认 rootfs 中的 init、动态库、设备节点和启动配置是否完整，而不只是看文件是否已经写入存储介质。



