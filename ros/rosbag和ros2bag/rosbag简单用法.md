## 1. 录制 bag

```bash
# 录制某个话题
rosbag record /imu_data

# 录制多个话题
rosbag record /imu_data /camera/left /camera/right

# 录制所有话题
rosbag record -a

# 指定文件名
rosbag record -O my_data.bag /imu_data /camera/left /camera/right
```

注意：**ROS1 的 rosbag 文件是一个单独的 `.bag` 文件**，不像 ROS2 那样是 `.db3` 目录。

---

## 2. 回放 bag

```bash
rosbag play my_data.bag
```

常见参数：

* `-l` 循环播放（loop）
* `-r 0.5` 以 0.5 倍速播放
* `-s 10` 从第 10 秒开始播放
* `-u 5` 只播放 5 秒钟

例如：

```bash
rosbag play my_data.bag -l -r 2.0
```

循环播放，并且加速两倍。

---

## 3. 查看 bag 信息

```bash
rosbag info my_data.bag
```

输出示例：

```
path:        my_data.bag
version:     2.0
duration:    2:34s (154s)
start:       Sep  9 2025 15:01:23.123
end:         Sep  9 2025 15:03:57.456
size:        45.3 MB
messages:    2354
compression: none [1/1 chunks]
types:       sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
topics:      /imu_data    1177 msgs    : sensor_msgs/Imu
             /camera/left 589 msgs     : sensor_msgs/Image
             /camera/right588 msgs     : sensor_msgs/Image
```

---

## 4. 过滤 bag（提取部分数据）

```bash
# 只保存 /imu_data
rosbag filter my_data.bag imu_only.bag "topic == '/imu_data'"

# 只保存前 10 秒
rosbag filter my_data.bag short.bag "t.secs - start.secs < 10"
```

---

## 5. 写 bag（编程方式）

在 ROS1 的 Python 里：

```python
import rosbag
from sensor_msgs.msg import Imu
import rospy

bag = rosbag.Bag('test.bag', 'w')

msg = Imu()
msg.header.stamp = rospy.Time.now()
bag.write('/imu_data', msg)

bag.close()
```

---

🔑 总结：

* **ROS1** → `.bag` 文件，用 `rosbag record/play/info/filter`。
* **ROS2** → `.db3` 目录，用 `ros2 bag record/play/info/convert`。
