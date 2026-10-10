
---

## 一、什么是软链接？

**软链接 ≈ Windows 的“快捷方式”**

* 软链接是一个**特殊文件**
* 文件内容是：**指向另一个文件或目录的路径**
* 访问软链接时，系统会自动跳转到目标文件

📌 本质：

> **软链接保存的是“路径”，不是文件本身**

---

## 二、创建软链接

```bash
ln -s 目标文件/目录  软链接名
```

### 示例

```bash
ln -s /home/user/data model_data
```
或
```bash
# 覆盖旧链接
ln -sf uEnvLubanCat3IO.txt uEnv.txt
```

结果：

```bash
model_data -> /home/user/data
```

### 目录也可以

```bash
ln -s /opt/ros/humble ~/ros_humble
```

---

## 三、软链接的核心特点（重点）

### ✅ 1. 可以跨文件系统

* 不同磁盘 / 不同分区 / WSL ↔ Linux
* **硬链接不行，软链接可以**

---

### ✅ 2. 可以指向目录

```bash
ln -s /path/to/dir link_dir
```

---

### ❌ 3. 目标不存在会“断链”

```bash
rm /home/user/data
ls model_data
# model_data -> (No such file or directory)
```

🔴 软链接还在，但**指向的路径没了**

---

### ❌ 4. 依赖路径（相对 / 绝对）

```bash
ln -s ../data data_link   # 相对路径
ln -s /abs/path/data data_link  # 绝对路径
```

📌 **推荐在工程里用相对路径**（更可移植）

---

## 四、查看软链接

```bash
ls -l
```

示例：

```bash
lrwxrwxrwx 1 user user  15 model_data -> /home/user/data
```

* 开头 `l`：link
* `->` 后是目标路径

---

## 五、删除软链接（不会影响原文件）

```bash
rm model_data
```

⚠️ **千万不要 `rm model_data/`**（有时会误删目标）

---

## 六、软链接 vs 硬链接（快速对比）

| 对比项   | 软链接          | 硬链接      |
| ----- | ------------ | -------- |
| 本质    | 路径引用         | inode 引用 |
| 跨文件系统 | ✅ 可以         | ❌ 不行     |
| 指向目录  | ✅ 可以         | ❌（一般禁止）  |
| 原文件删除 | ❌ 失效         | ✅ 仍存在    |
| 类似    | Windows 快捷方式 | 同一文件多个名字 |

---

## 七、典型使用场景（很重要）

### 🔹 1. 机器学习 / 模型管理（你现在很常用）

```bash
models/
├── yolov11.pt
└── latest.pt -> yolov11.pt
```

👉 更新模型只需改链接

---

### 🔹 2. ROS / ROS 2 开发（你这个场景特别多）

```bash
ln -s ~/nav2_ws/install ~/current_ws
```

* 切换工作空间
* 切换 map / config / weight 文件

---

### 🔹 3. WSL + Windows 混合开发

```bash
ln -s /mnt/c/Users/.../dataset ~/dataset
```

---

### 🔹 4. 版本切换

```bash
python -> python3.10
cuda -> cuda-12.1
```

---

## 八、软链接的坑（一定要记住）

⚠️ **1. 移动原文件，软链接会断**

```bash
mv data data2
# 原软链接失效
```

⚠️ **2. 打包 / 拷贝时可能变成“死链”**

```bash
cp -r project project_copy
# 软链接仍指向旧路径
```

⚠️ **3. Docker / ROS launch 时路径解析问题**

* 容器内路径 ≠ 宿主机路径

---

## 九、一句话记忆版

> **软链接 = 保存路径的“跳转文件”，像快捷方式，可跨文件系统，但怕目标消失**

---
