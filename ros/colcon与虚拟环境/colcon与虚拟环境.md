
---

##  使colcon build时使用的python指向虚拟环境的python

```bash
# 创建虚拟环境
python3 -m venv ~/ros_venv
source ~/ros_venv/bin/activate

# 在虚拟环境中安装colcon-common-extensions
pip install colcon-common-extensions

source /opt/ros/jazzy/setup.bash
source ~/ros_venv/bin/activate

which python
which -a python3
echo $PATH

which colcon

# 编译
colcon build 

```