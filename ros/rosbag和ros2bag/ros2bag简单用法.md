## 1. 录制 bag

```bash
# 录制某个话题
ros2 bag record /imu_data

# 录制多个话题
ros2 bag record /imu_data /camera/left /camera/right

# 录制所有话题
ros2 bag record -a

# 指定文件名（默认是 rosbag2_YYYY_MM_DD）
ros2 bag record -o my_bag /imu_data /camera/left /camera/right

# 指定输出格式db3
ros2 bag record -s sqlite3 -o my_bag /imu_data /camera/left /camera/right

# 后台录取（防止终端退出被杀死）
nohup ros2 bag record /imu_data &

```

录制出来的是一个目录，比如 `my_bag/`，里面会有：

* `metadata.yaml`
* `my_bag_0.db3`  ← 这个就是存数据的 SQLite 数据库文件

---

## 2. 回放 bag

```bash
# 回放
ros2 bag play my_bag

# 限速播放
ros2 bag play my_bag --rate 0.5

# 只回放某个话题
ros2 bag play my_bag --topics /imu_data
```

---

## 3. 查看 bag 信息

```bash
ros2 bag info my_bag
```

示例输出：

```
Files:             my_bag_0.db3
Bag size:          45.3 MiB
Storage id:        sqlite3
Duration:          2m34s
Start:             Sep  9 2025 15:01:23.123 (1631184083.123)
End:               Sep  9 2025 15:03:57.456 (1631184237.456)
Messages:          2354
Topic information: 
Topic: /imu_data    | Type: sensor_msgs/msg/Imu       | Count: 1177
Topic: /camera/left | Type: sensor_msgs/msg/Image     | Count: 589
Topic: /camera/right| Type: sensor_msgs/msg/Image     | Count: 588
```

---

## 4. 转换 bag 格式

ROS2 默认是 `.db3`（sqlite3），也支持 `mcap`。你可以转换：

```bash
ros2 bag convert -s mcap my_bag
```

会得到一个 `my_bag.mcap` 文件。

---

## 5. Python API (`rosbag2_py`)

你也可以在 Python 脚本里读写 bag（比如用来转到 ROS1）。

```python
import rosbag2_py

storage_options = rosbag2_py.StorageOptions(uri="my_bag", storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

while reader.has_next():
    (topic, data, t) = reader.read_next()
    print(f"Topic: {topic}, Time: {t}, Raw size: {len(data)}")
```

---
