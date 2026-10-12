# ROS2 aarch64 交叉编译概要

## 1 快速查阅

```sh

# 安装新包至sysroot
sudo chroot /opt/aarch64-sysroot
apt install <package_name>
exit

# 编译
colcon build \
    --build-base build_aarch64 --install-base install_aarch64 \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=../../aarch64-toolchain.cmake

# 下传编译产物
rsync -av install_aarch64/ orangepi@192.168.1.139:/home/orangepi/workspace/pl_vins/install_aarch64
```

运行sudo chroot /opt/aarch64-sysroot报错时运行以下代码
```sh
magic='\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\xb7\x00'
mask='\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff'
arch='aarch64'
interpreter="/usr/bin/qemu-$arch-static"
echo ":qemu-$arch:M::$magic:$mask:$interpreter:" | sudo tee /proc/sys/fs/binfmt_misc/register
```

然后查看是否真正进入了：`
```sh
# 查看cpu架构
uname -m

# 并且不能使用sudo，使用sudo时会报错
```

## 2 配置
### 安装常用工具
```sh
# 有些工具不是必须安装，比如tree、just
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    rsync \
    python3-pip \
    python3-venv \
    ccache \
    pkg-config \
    tree \
    vim \
    gdb \
    file \
    just
```

### 建立sysroot

```sh
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

sudo apt install debootstrap qemu-user-static binfmt-support
#更高版本执行：sudo apt install qemu-user-binfmt binfmt-support debootstrap

# 执行不带--foreign参数的命令会直接运行第一阶段和第二阶段，但如果主机是x86，则建议执行带--foreign参数的命令
sudo debootstrap --arch=arm64 noble /opt/aarch64-sysroot http://ports.ubuntu.com/

# --foreign跨架构  只执行第一阶段安装
sudo debootstrap --arch=arm64 \
    --foreign \
    noble \
    /opt/aarch64-sysroot \
    http://ports.ubuntu.com/

```
*ubuntu版本不一样需要修改对应的代号，比如ubuntu24->noble,ubuntu26->resolute*

注意，可能需要额外配置qemu
```bash
sudo cp /usr/bin/qemu-aarch64-static \
  /opt/aarch64-sysroot/usr/bin/
```

配置用户名
```bash
sudo chroot /opt/aarch64-sysroot /usr/bin/qemu-aarch64-static /bin/bash
#或 sudo cp /usr/bin/qemu-aarch64 /opt/aarch64-sysroot/usr/bin/

sudo chroot /opt/aarch64-sysroot

/debootstrap/debootstrap --second-stage


# 配置用户名
echo ubuntu-cross > /etc/hostname
cat /etc/apt/sources.list

cat >/etc/apt/sources.list <<EOF
deb http://ports.ubuntu.com/ubuntu-ports resolute main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports resolute-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports resolute-security main restricted universe multiverse
EOF

apt update

exit
```

### 配置sysroot环境

```sh
# 安装一些包的时候可能需要挂载目录
sudo mount --bind /dev /opt/aarch64-sysroot/dev
sudo mount --bind /dev/pts /opt/aarch64-sysroot/dev/pts
sudo mount -t proc /proc /opt/aarch64-sysroot/proc
sudo mount --rbind /sys /opt/aarch64-sysroot/sys

sudo chroot /opt/aarch64-sysroot

# 进入sysroot后
apt update

# 安装ROS2等

exit
```

也可在host中进行包安装

```sh
sudo dpkg --add-architecture arm64
sudo apt update
apt download libblas3:arm64
dpkg-deb -x package_name.deb /opt/aarch64-sysroot
```

注意，部分软件包适配[alternatives机制](https://wiki.debian.org/DebianAlternatives)，安装中可能出现符号链接异常的情况


# 安装cv-bridge的时候可以会报错说没有挂载文件系统
## 挂载文件系统
```bash
# 在sysroot外执行
sudo mount -t proc /proc /opt/aarch64-sysroot/proc
sudo mount -t sysfs /sys /opt/aarch64-sysroot/sys
sudo mount --rbind /dev /opt/aarch64-sysroot/dev
sudo mount --rbind /dev/pts /opt/aarch64-sysroot/dev/pts
```

## 卸载挂载
```bash
sudo umount /opt/aarch64-sysroot/proc
sudo umount /opt/aarch64-sysroot/sys
sudo umount -l /opt/aarch64-sysroot/dev/pts
sudo umount -l /opt/aarch64-sysroot/dev
```