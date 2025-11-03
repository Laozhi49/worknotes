
---

# 🧠 树莓派 systemd 自启动脚本总结大全

---

## 🐍 一、Python 脚本自启动

**目标：**
开机自动运行一个 Python 脚本（例如 `/home/pi/startup.py`）

### 1️⃣ 服务文件路径：

```bash
sudo nano /etc/systemd/system/startup_py.service
```

### 2️⃣ 内容模板：

```ini
[Unit]
Description=Python Startup Script
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/startup.py
WorkingDirectory=/home/pi
User=pi
Restart=always

[Install]
WantedBy=multi-user.target
```

### 3️⃣ 启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable startup_py.service
sudo systemctl start startup_py.service
```

✅ 适用：纯 Python 脚本、HTTP server、MQTT client 等
（例如：`python3 -m http.server`）

---

## 🐚 二、Bash 脚本自启动

**目标：**
运行 `/home/pi/startup.sh`

### 服务文件：

```bash
sudo nano /etc/systemd/system/startup_sh.service
```

### 内容：

```ini
[Unit]
Description=Bash Startup Script
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash /home/pi/startup.sh
WorkingDirectory=/home/pi
User=pi
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

⚙️ 如果脚本需要等待网络，可以在脚本开头加：

```bash
sleep 5
```

---

## 🦾 三、ROS 2 节点自启动

**目标：**
开机自动运行 ROS 2 节点，例如：

```bash
ros2 run my_package my_node
```

### 服务文件：

```bash
sudo nano /etc/systemd/system/ros2_node.service
```

### 内容模板：

```ini
[Unit]
Description=ROS 2 Node Startup
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/pi/ros2_ws/install/setup.bash && ros2 run my_package my_node"
Restart=on-failure
Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1

[Install]
WantedBy=multi-user.target
```

💡 说明：

* 使用 `/bin/bash -c` 执行多条命令（包括 `source`）
* 如果有自定义 workspace，别忘记 `source install/setup.bash`
* `Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1` 可防止 ROS 日志卡住输出

---

## 🌐 四、Flask / FastAPI / Web 服务类脚本

**目标：**
运行一个持续监听端口的服务脚本，比如 `/home/pi/webapp/app.py`

### 服务文件：

```bash
sudo nano /etc/systemd/system/webapp.service
```

### 内容：

```ini
[Unit]
Description=Flask Web Server
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/webapp
ExecStart=/usr/bin/python3 /home/pi/webapp/app.py
Restart=always

[Install]
WantedBy=multi-user.target
```

💡 如果你用虚拟环境：

```ini
ExecStart=/home/pi/venv/bin/python /home/pi/webapp/app.py
```

---

## 🪛 五、带参数或环境变量的脚本

例如你要运行带参数的脚本：

```bash
python3 /home/pi/task.py --mode test --port 8080
```

### 内容模板：

```ini
[Service]
ExecStart=/usr/bin/python3 /home/pi/task.py --mode test --port 8080
WorkingDirectory=/home/pi
User=pi
Restart=on-failure
Environment="MY_ENV=production"
```

可以加多个 `Environment=` 行设置环境变量。

---

## ⚙️ 六、共用的命令总结

| 操作         | 命令                                  |
| ---------- | ----------------------------------- |
| 重载 systemd | `sudo systemctl daemon-reload`      |
| 启动服务       | `sudo systemctl start xxx.service`  |
| 停止服务       | `sudo systemctl stop xxx.service`   |
| 设置开机启动     | `sudo systemctl enable xxx.service` |
| 查看运行状态     | `systemctl status xxx.service`      |
| 查看日志       | `journalctl -u xxx.service -f`      |

---

## ✅ 七、小技巧与经验

* 加 `Restart=always` 可在脚本崩溃后自动重启
* 若脚本依赖网络，`After=network.target` 或 `After=network-online.target`
* 确保脚本路径无权限问题（`chmod +x`）
* 永远放在 `/etc/systemd/system/` 而不是 `/lib/systemd/system/`（系统更新安全）
* 修改后必须运行：

  ```bash
  sudo systemctl daemon-reload
  ```

---

## 🧩 总览对照表

| 类型             | ExecStart 示例                                | 是否需 source 环境 | 推荐  |
| -------------- | ------------------------------------------- | ------------- | --- |
| Python 脚本      | `/usr/bin/python3 /path/script.py`          | 否             | ✅✅  |
| Bash 脚本        | `/bin/bash /path/script.sh`                 | 否             | ✅   |
| ROS 2 节点       | `/bin/bash -c "source ... && ros2 run ..."` | ✅ 是           | ✅✅✅ |
| Flask / Web 服务 | `/usr/bin/python3 /path/app.py`             | 否             | ✅✅  |
| 带虚拟环境          | `/home/pi/venv/bin/python ...`              | 否             | ✅   |

---