# catkin_ws

本目录是项目唯一的 Catkin 工作空间，所有 ROS 包、控制器、规划器、定位节点和
Gazebo 地图都位于 `src` 下。

## 构建

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make_isolated
source devel_isolated/setup.bash
```

如仅构建新增的桥接、悬停和地图包，也可以使用常规 `catkin_make`；完整工程包含
不同构建类型的第三方包，推荐使用 `catkin_make_isolated`。

## 森林地图测试

50×50×5 m 森林地图包含 72 棵树，无人机默认从西侧边缘
`(-21.0, 0.0, 0.2)` 出生：

```bash
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch ego_gazebo_worlds px4_mid360_forest.launch
```

仅启动地图：

```bash
roslaunch ego_gazebo_worlds forest_map_only.launch
```

重新生成和校验森林地图：

```bash
python3 src/ego_gazebo_worlds/scripts/generate_forest_world.py
python3 src/ego_gazebo_worlds/scripts/generate_forest_world.py --validate-only
```

原 50×50×4 m 障碍物地图仍可使用：

```bash
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
```

## 主要运行链路

1. 启动 PX4 SITL、Gazebo 和 MID360；
2. 启动 `fast_lio` 的 MID360 配置；
3. 启动 `fastlio_px4_bridge` 或 `camera_pose_node`；
4. 启动 EGO-Planner；
5. 启动 SE3 控制器或 `offboard_hover`；
6. 在 RViz 中发布导航目标。

PX4 的 `iris_mid360` 和 `Mid360` Gazebo 模型需要预先安装到 PX4 的
`Tools/sitl_gazebo/models` 目录。
