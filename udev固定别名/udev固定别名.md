
---

# 🧭 udev 基础与固定别名笔记

---

## 一、什么是 udev？

**udev** 是 Linux 下的 **设备管理守护进程**，负责管理 `/dev/` 目录中的所有设备节点。
当插入或拔出设备（如 USB、串口、摄像头）时，udev 会自动：

* 创建或删除对应的 `/dev/xxx` 文件；
* 设置设备权限；
* 应用用户自定义规则（例如固定命名）。

📌 udev 的配置文件位于：

```
/etc/udev/rules.d/
```

系统默认规则在：

```
/lib/udev/rules.d/
```

---

## 二、udev 规则基本语法

每条规则的格式是：

```
匹配条件, 操作
```

例如：

```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="wheeltec_IMU"
```

### 常用匹配字段：

| 字段                 | 含义                      | 示例                  |
| ------------------ | ----------------------- | ------------------- |
| `KERNEL`           | 匹配设备名（如 ttyUSB*、video*） | `KERNEL=="ttyACM*"` |
| `SUBSYSTEM`        | 匹配设备类型（usb、tty、block 等） | `SUBSYSTEM=="tty"`  |
| `ATTRS{idVendor}`  | 厂商ID（VID）               | `"1a86"`            |
| `ATTRS{idProduct}` | 产品ID（PID）               | `"55d4"`            |
| `ATTRS{serial}`    | 设备序列号                   | `"0003"`            |
| `KERNELS`          | 上级设备路径（物理 USB 口）        | `"2-1"`             |

### 常用操作字段：

| 操作        | 含义         | 示例                        |
| --------- | ---------- | ------------------------- |
| `MODE`    | 权限设置       | `MODE:="0777"`            |
| `GROUP`   | 设置所属用户组    | `GROUP:="dialout"`        |
| `SYMLINK` | 创建符号链接（别名） | `SYMLINK+="wheeltec_IMU"` |

---

## 三、查看设备信息（调试）

### 1️⃣ 查看当前系统识别的设备

```bash
ls /dev/tty*
```

### 2️⃣ 查询设备的 udev 属性（最重要）

```bash
udevadm info -a -n /dev/ttyACM0
```

🔍 重点关注：

```
ATTRS{idVendor}
ATTRS{idProduct}
ATTRS{serial}
KERNELS
```

或者直接过滤：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial|KERNELS"

udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial|KERNELS"
```

---

## 四、创建固定别名规则（最常用）

### ✅ 示例：给 IMU 固定别名 `/dev/wheeltec_IMU`

```bash
sudo nano /etc/udev/rules.d/wheeltec_imu.rules
```

写入内容：

```bash
# CP2102驱动
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"

# CH340驱动
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"

```

### 也可以创建wheeltec_IMU.bash，写入如下内容：
```bash
# CP2102驱动
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"' >/etc/udev/rules.d/wheeltec_imu_ACM.rules

# CH340驱动
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"' >/etc/udev/rules.d/wheeltec_imu_USB.rules

# 安全方式重新加载 udev 规则
udevadm control --reload-rules
udevadm trigger
echo "wheeltec udev rules reloaded successfully!"
```

> 注意：每台设备的 `serial` 都不同，要根据实际设备值修改。

保存退出。

运行：
```bash
sudo bash wheeltec_IMU.bash
```

---

## 五、重新加载规则并立即生效

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

也可以更彻底地重启服务：

```bash
sudo service udev restart
```

---

## 六、验证是否生效

1️⃣ 拔插设备
2️⃣ 查看是否生成了别名：

```bash
ll /dev

ls -l /dev/wheeltec_IMU
```

应该看到类似：

```
lrwxrwxrwx 1 root root 7 Oct 20 15:00 /dev/wheeltec_IMU -> ttyACM0
```

✅ 表示 `/dev/ttyACM0` 的别名是 `/dev/wheeltec_IMU`，系统或程序都可以用后者访问。

---

## 七、进阶用法（多个相同设备）

如果两个设备的 `idVendor`、`idProduct` 一样，但 `serial` 不同，可以分别命名：

```bash
# 第一个设备
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", SYMLINK+="imu_left"

# 第二个设备
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="597B019678", SYMLINK+="imu_right"
```

这样即使系统交换顺序（ACM0 ↔ ACM1），程序也能通过 `/dev/imu_left`、`/dev/imu_right` 固定访问。

---

## 八、物理端口识别方案（无序列号时）

有的设备没有 `serial`，可以用 USB 端口路径区分：

```bash
KERNEL=="ttyUSB*", KERNELS=="2-2", SYMLINK+="gps_port1"
KERNEL=="ttyUSB*", KERNELS=="2-3", SYMLINK+="gps_port2"
```

用命令查看端口路径：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
```

---

## 九、调试技巧

* 查看规则是否被加载：

  ```bash
  udevadm info -q all -n /dev/ttyACM0
  ```
* 触发单个设备：

  ```bash
  sudo udevadm trigger -v -n /dev/ttyACM0
  ```
* 实时监控 udev 事件：

  ```bash
  sudo udevadm monitor --property
  ```

---

## 🔟 常见问题总结

| 问题     | 原因                | 解决                                                  |
| ------ | ----------------- | --------------------------------------------------- |
| 规则无效   | 文件名没以 `.rules` 结尾 | 确保在 `/etc/udev/rules.d/` 下且以 `.rules` 结尾            |
| 权限不足   | MODE/GROUP 没设置    | `MODE:="0777", GROUP:="dialout"`                    |
| 多个设备冲突 | serial 一样         | 改匹配条件或换物理口                                          |
| 别名不出现  | 没 reload 规则       | `udevadm control --reload-rules && udevadm trigger` |

---

## 🧠 总结口诀

> **查属性 → 写规则 → reload → 验证别名**

简化为 4 步：

1️⃣ 查设备信息

```bash
udevadm info -a -n /dev/ttyACM0
```

2️⃣ 写规则文件

```bash
sudo nano /etc/udev/rules.d/xxx.rules
```

3️⃣ 重新加载规则

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

4️⃣ 验证

```bash
ls -l /dev/your_alias
```

---

