啊哈 ✅，懂了～
你的意思是：

* **ROS2\_StudyNote** 作为一个大目录
* 在里面新建一个 **全新的工作区 bt\_jazzy\_ws**
* 这个工作区只使用 **ROS 2 Jazzy 自带的 behaviortree\_cpp**（不自己源码覆盖）
* demo 工程按照 **XML 文件 + 节点类 + main.cpp 分离** 的形式写

这样不会跟你之前 `bt_ws` 里的源码冲突 👍

---

## 操作步骤

### 1️⃣ 新建工作区

```bash
cd ~/ROS2_StudyNote
mkdir -p bt_jazzy_ws/src
cd bt_jazzy_ws/src
```

### 2️⃣ 创建 demo 包

```bash
ros2 pkg create --build-type ament_cmake bt_demo --dependencies behaviortree_cpp
```

目录结构会是：

```
bt_jazzy_ws/src/bt_demo/
├── CMakeLists.txt
├── package.xml
├── include/bt_demo/
│   └── nodes.hpp
├── src/
│   ├── main.cpp
│   └── nodes.cpp
└── trees/
    └── demo_tree.xml
```

---

## 代码部分

我帮你写好和 Jazzy 自带 BT 库兼容的版本 👇

### `include/bt_demo/nodes.hpp`

```cpp
#ifndef NODES_HPP
#define NODES_HPP

#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

// 条件节点：检查电池是否充足
class BatteryOK : public BT::ConditionNode
{
public:
  BatteryOK(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  bool battery_ok_;
};

// 行为节点：执行工作
class DoWork : public BT::SyncActionNode
{
public:
  DoWork(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 行为节点：提示电池低
class LowBattery : public BT::SyncActionNode
{
public:
  LowBattery(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

#endif
```

---

### `src/nodes.cpp`

```cpp
#include "bt_demo/nodes.hpp"

// ========== BatteryOK ==========
BatteryOK::BatteryOK(const std::string& name, const BT::NodeConfig& config)
: BT::ConditionNode(name, config), battery_ok_(true) {}

BT::PortsList BatteryOK::providedPorts() { return {}; }

BT::NodeStatus BatteryOK::tick()
{
  if (battery_ok_)
  {
    std::cout << "[BatteryOK] Battery is sufficient." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::cout << "[BatteryOK] Battery is LOW!" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

// ========== DoWork ==========
DoWork::DoWork(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList DoWork::providedPorts() { return {}; }

BT::NodeStatus DoWork::tick()
{
  std::cout << "[DoWork] Doing some work..." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// ========== LowBattery ==========
LowBattery::LowBattery(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList LowBattery::providedPorts() { return {}; }

BT::NodeStatus LowBattery::tick()
{
  std::cout << "[LowBattery] Warning: Battery is too low!" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
```

---

### `src/main.cpp`

```cpp
#include "bt_demo/nodes.hpp"

int main(int argc, char** argv)
{
  BT::BehaviorTreeFactory factory;

  // 注册节点
  factory.registerNodeType<BatteryOK>("BatteryOK");
  factory.registerNodeType<DoWork>("DoWork");
  factory.registerNodeType<LowBattery>("LowBattery");

  // 从 XML 文件加载
  auto tree = factory.createTreeFromFile(BT_TREE_PATH);

  std::cout << "Ticking the tree..." << std::endl;
  auto status = tree.tickOnce();
  std::cout << "Tree finished with status: " << BT::toStr(status) << std::endl;

  return 0;
}
```

---

### `trees/demo_tree.xml`

```xml
<root main_tree_to_execute="MainTree" BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Fallback>
      <Sequence>
        <BatteryOK/>
        <DoWork/>
      </Sequence>
      <LowBattery/>
    </Fallback>
  </BehaviorTree>
</root>
```

---

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(bt_demo)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# 加入 include 路径（推荐 ament 的写法）
target_include_directories(bt_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include)

add_executable(bt_main src/main.cpp src/nodes.cpp)
ament_target_dependencies(bt_main behaviortree_cpp)

install(TARGETS bt_main
  DESTINATION lib/${PROJECT_NAME})

# 安装 XML 文件
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees)

# 宏定义树文件的路径
target_compile_definitions(bt_main
  PRIVATE BT_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/trees/demo_tree.xml")

ament_package()
```

---

## 编译运行

```bash
cd ~/ROS2_StudyNote/bt_jazzy_ws
colcon build --packages-select bt_demo
source install/setup.bash
ros2 run bt_demo bt_main
```

输出大概是：

```
Ticking the tree...
[BatteryOK] Battery is sufficient.
[DoWork] Doing some work...
Tree finished with status: SUCCESS
```

---

要不要我帮你把 `BatteryOK` 里的 `battery_ok_` 改成一个 **ROS 2 参数**（例如运行时 `ros2 run bt_demo bt_main --ros-args -p battery_ok:=false` 就能触发 `LowBattery`）？
