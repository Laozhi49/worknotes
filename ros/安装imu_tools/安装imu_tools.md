源码:https://github.com/CCNYRoboticsLab/imu_tools/tree/jazzy

源码安装imu_tools
```bash
cd ~/ros_ws/src
rm -rf imu_tools
git clone -b <YOUR_ROSDISTO> https://github.com/CCNYRoboticsLab/imu_tools.git
cd ..
rm -rf build install log
colcon build --packages-up-to imu_tools
```
