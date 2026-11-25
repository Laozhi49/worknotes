# Navigation2 主要功能包功能解释

## nav2_behavior_tree
* 提供nav2_behavior_tree::BehaviorTreeEngine以及bt_action、bt_action_server等基类的实现；
* 提供行为树插件（包括action、condition、control和decorator）；
* bt_action_server是提供给navigator用的；
* bt_action是提供给行为树里的插件用的，比如ComputePathToPose、FollowPath；这些插件都有个特点即作为Action client，因为bt_action基类会创建对应名字的action client，并在tick的时候向action server发送请求，同时抢占机制也已经实现；

## nav2_bt_navigator
* 提供navigate_to_pose Action;
* 提供navigate_through_pose Action;
* 两个action都继承自nav2_core::BehaviorTreeNavigator，而这个基类会创建Action server并绑定onGoalReceived、onLoop、onPreempt、onCompletion这些函数；也就是说会同时拥有action server和action client;
* nav2_core::BehaviorTreeNavigator创建的action server继承自nav2_behavior_tree::BtActionServer；
* 每个nav2_behavior_tree::BtActionServer都会创建一个nav2_behavior_tree::BehaviorTreeEngine和一块Blackboard，同时，nav2_behavior_tree::BtActionServer创建的ActionServer绑定的executeCallback函数会调用nav2_behavior_tree::BehaviorTreeEngine的run()函数，因此整棵行为树得以被周期tick；

## nav2_behavior
* 提供运动时的行为树插件，比如back_up、spin、wait等； 

## nav2_planner
* 提供planner server：compute_path_to_pose和compute_path_through_poses；
* action server根据插件去选择调用具体的路径规划算法函数；
* 创建并管理 global_costmap；
* 全局路径规划算法；

## nav2_navfn_planner
* 全局路径规划的具体实现（A*算法规划路径）；

## nav2_controller
* 提供follow_path server；
* action server根据插件去选择调用具体的机器人控制算法；
* 创建并管理 local_costmap；

## nav2_mppi_controller
* 运动控制算法的具体实现；
* 局部轨迹优化；

## nav2_dwb_controller
* 运动控制算法的具体实现；
* 局部轨迹优化；

## nav2_regulated_pure_pursuit_controller
* 运动控制算法的具体实现；

## nav2_map_server
* map_sever: 从yaml文件加载地图并发布;
* map_saver: 将地图保存为yaml文件;

## nav2_costmap_2d
* 提供底层地图数据结构实现：Costmap2D（costmap_2d.cpp）；
* 各个地图层的基类实现：CostmapLayer（costmap_layer.hpp）；
* 多层地图融合管理实现：LayeredCostmap（layered_costmap.cpp）；
* 主要有4个层：static_layer、obstacle_layer、inflation_layer和voxel_layer；
* static_layer    ：订阅nav2_map_server发布的/map并把已有的静态地图加入 Costmap；
* obstacle_layer  ：订阅/scan，通过 ObservationBuffer 管理传感器观测数据，并把观测数据转换成障碍物标记和清除操作；
* voxel_layer     ：订阅/scan（也可以是点云数据），维护一个三维体素结构，而不是只维护二维障碍物信息，但最终仍然是给二维 Costmap 使用的，它会根据一定高度范围内的体素占用情况，投影成二维障碍物信息；
* inflation_layer ：在障碍物周围膨胀主地图；

## nav2_amcl
* Adaptive Monte Carlo Localization，自适应蒙特卡洛定位；
* 已经有一张已知的地图，机器人通过激光雷达和里程计等传感器数据，估计自己当前在地图中的位置和姿态；
* 粒子滤波：输入：地图、激光雷达、里程计，输出：机器人在 map 坐标系中的位姿；
* 发布map → odom TF；