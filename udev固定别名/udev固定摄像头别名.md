
---

# 🧩 **一、USB 摄像头固定设备名的两种方式（总览）**

| 方法                                        | 能否区分两个完全相同的摄像头   | 推荐场景              | 稳定性       |
| ----------------------------------------- | ---------------- | ----------------- | --------- |
| **方法 A：使用摄像头自身属性（Vendor、Product、Serial）** | ❌ 不行（序列号相同就无法区分） | 摄像头有唯一序列号         | ⭐⭐⭐       |
| **方法 B：使用 USB 物理端口路径（DEVPATH）**           | ✅ 可以             | 摄像头没有唯一 ID，多个完全相同 | ⭐⭐⭐⭐⭐（最稳） |

---

# 🧩 **二、方法 A：不使用物理端口（靠设备自身属性）**

适用于：
✔ 摄像头有唯一的 Serial Number
✔ 或只有一个摄像头

你可以读取属性：

```bash
udevadm info --query=all --name=/dev/video0 | grep -E "ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID"
```

示例规则（按你的格式）：

```bash
echo 'KERNEL=="video*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2993", ATTRS{idProduct}=="0858", ATTRS{serial}=="SOME_SERIAL", MODE:="0777", GROUP:="video", SYMLINK+="camera_front"' >/etc/udev/rules.d/camera_front.rules
```

**缺点**
如果两个摄像头：

* Vendor ID 相同
* Product ID 相同
* Serial 相同（很多国产摄像头都是同一个）

就无法区分。

---

# 🧩 **三、方法 B：使用 USB 物理端口（DEVPATH）——区分相同摄像头的“终极方法”**

适用于：
✔ 摄像头型号一样
✔ 序列号也一样
✔ 插在不同 USB 口

这是你当前摄像头的情况。

你可以查看摄像头的物理路径：

```bash
udevadm info --query=all --name=/dev/video0 | grep DEVPATH
```

你得到：

### 摄像头 1（video0）

```
/usb3/3-2/
```

### 摄像头 2（video2）

```
/usb3/3-4/
```

于是创建规则（按你的 echo 写法）：

---

## **左摄像头（USB 口：3-2）**

```bash
echo 'KERNEL=="video*", SUBSYSTEM=="video4linux", ENV{DEVPATH}=="*/usb3/3-2/*", MODE:="0777", GROUP:="video", SYMLINK+="camera_left"' >/etc/udev/rules.d/camera_left.rules
```

## **右摄像头（USB 口：3-4.1）**

```bash
echo 'KERNEL=="video*", SUBSYSTEM=="video4linux", ENV{DEVPATH}=="*/usb3/3-4/*", MODE:="0777", GROUP:="video", SYMLINK+="camera_right"' >/etc/udev/rules.d/camera_right.rules
```

---

# 🧩 **四、让规则立即生效**

```bash
udevadm control --reload-rules
udevadm trigger
echo "camera udev rules reloaded successfully!"
```

---

# 🧩 **五、测试固定是否成功**

```bash
ls -l /dev/camera_left
ls -l /dev/camera_right
#或
ll /dev
```

---

# 🧩 **六、最终总结**

### ✔ 如果你的摄像头有唯一序列号 → 用 ATTRS{idVendor}/ATTRS{idProduct}/ATTRS{serial}

### ✔ 如果你的摄像头序列号完全一样（多数国产设备） → 必须用 USB 物理端口（ENV{DEVPATH}）

你的摄像头属于：

* **ID_VENDOR 一样**
* **ID_PRODUCT 一样**
* **ID_SERIAL 一样**

因此 **只能使用物理 USB 端口路径 DEVPATH 来区分**，否则永远会混。

---