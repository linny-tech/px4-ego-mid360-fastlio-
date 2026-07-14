# PX4 + EGO-Planner + MID360 + FAST-LIO

本仓库用于在 Gazebo Classic 11 中验证 PX4、Livox MID360、FAST-LIO 与
EGO-Planner 的完整仿真链路。所有 ROS 源码和地图均统一放在
[`catkin_ws`](catkin_ws) 工作空间中。

## 工作空间内容

- `FASTLIO`：MID360 点云与惯导里程计；
- `planner`：EGO-Planner、地图、轨迹优化及消息包；
- `controller`：SE3 控制器及控制消息；
- `camera_pose_node`：FAST-LIO 位姿到 PX4 视觉位姿转换；
- `fastlio_px4_bridge`：FAST-LIO 与 PX4 坐标/里程计桥接；
- `offboard_hover`：PX4 Offboard 起飞与悬停节点；
- `ego_gazebo_worlds`：50×50×4 m 原测试地图和 50×50×5 m 森林地图。

详细构建和启动说明见 [`catkin_ws/README.md`](catkin_ws/README.md)。

## 目标环境

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11
- PX4 SITL
