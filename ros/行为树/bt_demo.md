好嘞 👍 那我给你写一个最小可跑的 **ROS 2 Jazzy + BehaviorTree.CPP v4** 示例包。
这个包里包含：

* 一个自定义动作节点 `SaySomething`
* 一个 XML 行为树
* `CMakeLists.txt` 和 `package.xml` 配置

---

## 1. 创建 ROS2 包

在你的工作区 `~/ros2_ws/src` 里新建一个包：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_bt_demo
```

---

## 2. 目录结构

```
my_bt_demo/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── say_something.cpp
│   └── main.cpp
└── trees/
    └── demo_tree.xml
```

---

## 3. 源码文件

### `src/say_something.cpp`

```cpp
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "SaySomething: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// 方便在 main 里注册
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SaySomething>("SaySomething");
}
```

---

### `src/main.cpp`

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
  BT::BehaviorTreeFactory factory;

  // 加载插件（say_something.cpp 里注册的）
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("say_something"));

  // 从 XML 文件加载行为树
  auto tree = factory.createTreeFromFile("trees/demo_tree.xml");

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
```

---

### `trees/demo_tree.xml`

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <SaySomething message="Hello, BehaviorTree!"/>
      <SaySomething message="Running in ROS2 Jazzy"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 4. CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_bt_demo)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# 插件库
add_library(say_something SHARED src/say_something.cpp)
ament_target_dependencies(say_something behaviortree_cpp)

# 可执行程序
add_executable(bt_main src/main.cpp)
ament_target_dependencies(bt_main behaviortree_cpp_v4)
target_link_libraries(bt_main say_something)

install(TARGETS
  bt_main
  say_something
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees)

ament_package()
```

---

## 5. package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_bt_demo</name>
  <version>0.0.1</version>
  <description>Minimal BehaviorTree.CPP v4 demo for ROS 2 Jazzy</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>behaviortree_cpp</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 6. 编译运行

回到工作区编译：

```bash
cd ~/ros2_ws
colcon build --packages-select my_bt_demo
source install/setup.bash
```

运行：

```bash
ros2 run my_bt_demo bt_main
```

输出应该是：

```
SaySomething: Hello, BehaviorTree!
SaySomething: Running in ROS2 Jazzy
```

---

要不要我帮你扩展一下，让这个 **行为树节点调用 ROS 2 Action（比如 Nav2 的 NavigateToPose）**，这样就能和导航直接结合？
