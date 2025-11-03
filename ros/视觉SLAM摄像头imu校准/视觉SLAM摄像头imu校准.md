参考：[Calibrating Intel RealSense Cameras](https://github.com/ethz-asl/kalibr/wiki/Calibrating-Intel-RealSense-Cameras)

***开始之前，陀螺仪的3轴尽量与相机的3轴对齐，即 Z轴往前、Y轴往下、X轴往右***

---
环境：
* Ubuntu20.04
* ROS1 Noetic
---
使用的关键工具：  
* kalibr: https://github.com/ethz-asl/kalibr  
* allan_variance_ros: https://github.com/ori-drs/allan_variance_ros
---
## 1、使用allan_variance_ros计算陀螺仪的噪声
参考：[Allan Variance ROS](https://github.com/ori-drs/allan_variance_ros)

1.编译
```bash
# ~/catkin_ws/src/allan_variance_ros
# 在工作目录运行以下代码编译
catkin build allan_variance_ros
source devel/setup.bash
```

2.使用

*准备：请先录制一个至少3小时的imu静置的bag*
```bash
# terminal 1
roscore

# terminal 2
# 按时间戳重组ROS信息
rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag

# 运行allan_variance_ros计算工具（config文件参考config文件夹里的yaml文件）
rosrun allan_variance_ros allan_variance [path_to_folder_containing_bag] [path_to_config_file]

# 可视化绘图并获取参数
rosrun allan_variance_ros analysis.py --data allan_variance.csv
```
*最终会在工作目录生成一个imu.yaml文件，此文件里的topic和update_rate需要自己手动修改，并可直接用于Kalibr*

## 2、使用kalibr标定相机内参
参考：[Multiple camera calibration](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)

*校准相机的视频频率推荐为4HZ，时长1分钟左右，同时推荐使用 Aprilgrid 来校准而不是棋盘格*

Aprilgrid文件下载地址：https://github.com/ethz-asl/kalibr/wiki/downloads

example command:
```bash
rosrun kalibr kalibr_calibrate_cameras --target april_6x6.yaml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --bag cam_april.bag --bag-freq 4.0
```


## 3、使用kalibr进行相机和imu的联合标定
参考：[Camera IMU calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)

*录制的bag应参考[Camera IMU calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)*

可以运行以下命令进行联合标定
```bash
kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] --imu [imu.yaml] --target [target.yaml]
```

example command:
```bah
rosrun kalibr kalibr_calibrate_imu_camera --target april_6x6.yaml --imu imu.yaml --imu-models calibrated --cam cam_april-camchain.yaml --bag imu_april.bag
```