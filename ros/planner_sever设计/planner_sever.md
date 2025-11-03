# Planner Server 功能包实现

下面我将为你创建一个名为 `planner_server` 的功能包，它继承自生命周期节点并支持组件化，实现地图订阅、路径规划 Action 服务等功能。

## 1. 创建功能包

```bash
ros2 pkg create --build-type ament_cmake planner_server \
  --dependencies rclcpp rclcpp_lifecycle rclcpp_components nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
```

## 2. 功能包结构

```
planner_server/
├── CMakeLists.txt
├── include/planner_server
│   └── planner_server.hpp
├── src
│   ├── planner_server.cpp
│   └── main.cpp
├── action
│   └── ComputePath.action
└── package.xml
```

## 3. 实现代码

### ComputePath.action

```action
# Request
geometry_msgs/Point target_point
---
# Result
nav_msgs/Path path
---
# Feedback
float32 completion_percentage
```

### planner_server.hpp

```cpp
#ifndef PLANNER_SERVER_HPP
#define PLANNER_SERVER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "planner_server/action/compute_path.hpp"

class PlannerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ComputePath = planner_server::action::ComputePath;
  using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle node callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePath::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleComputePath> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleComputePath> goal_handle);
  void computePlanToPose(const std::shared_ptr<GoalHandleComputePath> goal_handle);

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::string global_frame_;
  std::string robot_frame_;
};

#endif  // PLANNER_SERVER_HPP
```

### planner_server.cpp

```cpp
#include "planner_server/planner_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::placeholders;

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("planner_server", options),
  global_frame_("map"),
  robot_frame_("base_link")
{
  this->declare_parameter("global_frame", global_frame_);
  this->declare_parameter("robot_frame", robot_frame_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring planner server...");
  
  // Get parameters
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create action server
  action_server_ = rclcpp_action::create_server<ComputePath>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::handleGoal, this, _1, _2),
    std::bind(&PlannerServer::handleCancel, this, _1),
    std::bind(&PlannerServer::handleAccepted, this, _1));
    
  // Create publisher (not activated yet)
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "global_path", rclcpp::SystemDefaultsQoS());
    
  // Create subscriber
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerServer::mapCallback, this, _1));
    
  RCLCPP_INFO(get_logger(), "Planner server configured.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating planner server...");
  
  // Activate publisher
  path_pub_->on_activate();
  
  RCLCPP_INFO(get_logger(), "Planner server activated.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating planner server...");
  
  // Deactivate publisher
  path_pub_->on_deactivate();
  
  RCLCPP_INFO(get_logger(), "Planner server deactivated.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PlannerServer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  RCLCPP_DEBUG(get_logger(), "Received new map");
}

rclcpp_action::GoalResponse PlannerServer::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ComputePath::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerServer::handleCancel(
  const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerServer::handleAccepted(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  std::thread{std::bind(&PlannerServer::computePlanToPose, this, _1), goal_handle}.detach();
}

void PlannerServer::computePlanToPose(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ComputePath::Result>();
  auto feedback = std::make_shared<ComputePath::Feedback>();
  
  // Check if map is available
  if (!current_map_) {
    RCLCPP_ERROR(get_logger(), "No map received yet");
    goal_handle->abort(result);
    return;
  }
  
  // Get current robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
    robot_pose.header.stamp = this->now();
    robot_pose.header.frame_id = global_frame_;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;
    robot_pose.pose.orientation = transform.transform.rotation;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "TF exception: %s", ex.what());
    goal_handle->abort(result);
    return;
  }
  
  // Create empty path (replace this with your actual planner)
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;
  
  // Add start pose
  path.poses.push_back(robot_pose);
  
  // Add goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = path.header;
  goal_pose.pose.position = goal->target_point;
  goal_pose.pose.orientation.w = 1.0;
  path.poses.push_back(goal_pose);
  
  // Publish path
  path_pub_->publish(path);
  
  // Set result
  result->path = path;
  goal_handle->succeed(result);
  
  RCLCPP_INFO(get_logger(), "Path planning completed");
}

RCLCPP_COMPONENTS_REGISTER_NODE(PlannerServer)
```

### main.cpp

```cpp
#include "planner_server/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerServer>(rclcpp::NodeOptions());
  
  // For lifecycle nodes, manually trigger transition to active
  node->configure();
  node->activate();
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(planner_server)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate actions
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputePath.action"
)

# Include directories
include_directories(include)

# Library
add_library(planner_server_lib SHARED
  src/planner_server.cpp
)
target_include_directories(planner_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_server_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

# Register as component
rclcpp_components_register_nodes(planner_server_lib "PlannerServer")

# Executable
add_executable(planner_server_node
  src/main.cpp
)
target_link_libraries(planner_server_node
  planner_server_lib
)
ament_target_dependencies(planner_server_node
  rclcpp
  rclcpp_lifecycle
)

# Install
install(TARGETS
  planner_server_lib
  planner_server_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>planner_server</name>
  <version>0.1.0</version>
  <description>Lifecycle-based path planning server with action interface</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>rclcpp_action</depend>
  <depend>rclcpp_components</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>rosidl_default_runtime</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <rclcpp_components>
      <plugin>planner_server::PlannerServer</plugin>
    </rclcpp_components>
  </export>
</package>
```

## 4. 使用说明

### 构建功能包

```bash
colcon build --packages-select planner_server
source install/setup.bash
```

### 运行方式

#### 作为独立节点运行

```bash
ros2 run planner_server planner_server_node
```

#### 作为组件运行

```bash
ros2 run rclcpp_components component_container
# 另一个终端
ros2 component load /ComponentManager planner_server PlannerServer
```

### 测试 Action 服务

```bash
ros2 action send_goal /compute_path_to_pose planner_server/action/ComputePath "{target_point: {x: 5.0, y: 3.0, z: 0.0}}"
```

## 5. 功能说明

1. **生命周期管理**：
   - 继承 `rclcpp_lifecycle::LifecycleNode`
   - 实现 `on_configure`, `on_activate`, `on_deactivate` 等回调

2. **组件化支持**：
   - 使用 `RCLCPP_COMPONENTS_REGISTER_NODE` 宏注册
   - 可通过组件管理器动态加载

3. **核心功能**：
   - 订阅 `/map` 话题获取地图数据
   - 提供 `ComputePath` Action 服务
   - 发布规划路径到 `/global_path` 话题

4. **TF 集成**：
   - 使用 `tf2` 获取机器人当前位置
   - 支持自定义全局帧和机器人帧参数

这个实现提供了一个框架，你可以替换 `computePlanToPose` 方法中的简单路径生成逻辑，集成你实际使用的路径规划算法（如A*、RRT等）。