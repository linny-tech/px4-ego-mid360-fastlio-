# PX4 + EGO-Planner + MID360 + FAST-LIO 仿真工作空间

本仓库使用 Git submodule 管理各个 ROS 功能包。每个功能包都可以独立克隆，也可以一次性递归克隆完整 `catkin_ws`。

## 克隆完整工作空间

```bash
git lfs install
git clone --recurse-submodules https://github.com/linny-tech/px4-ego-mid360-fastlio-.git
```

如果已经克隆了总仓库，请执行：

```bash
git submodule update --init --recursive
```

## 三角坡屋顶住宅小区地图

仅启动 50×50×10 米住宅地图：

```bash
roslaunch ego_gazebo_worlds residential_map_only.launch
```

启动 PX4、MID360 和住宅地图：

```bash
roslaunch ego_gazebo_worlds px4_mid360_residential.launch
```

地图为 50×50×10 米，住宅高度在 4–10 米之间错落排列；三角坡屋顶和落地支柱均使用长方体，不含圆柱。无人机默认从西侧边缘 `(-22, 0, 0.2)` 起飞，中间道路保持贯通。

## 弯曲狭窄隧道地图

仅启动 S 形隧道地图：

```bash
roslaunch ego_gazebo_worlds tunnel_map_only.launch
```

启动 PX4、MID360 和隧道地图：

```bash
roslaunch ego_gazebo_worlds px4_mid360_tunnel.launch
```

隧道位于 50×50×10 米范围内，包含 8 段、7 个明显转弯、低顶和侧壁凸起。无人机默认从 `(-22, -8, 0.3)` 起飞。

## 独立功能包仓库

- [camera_pose_node](https://github.com/linny-tech/camera_pose_node)
- [ego-planner](https://github.com/linny-tech/ego-planner)
- [FAST_LIO](https://github.com/linny-tech/FAST_LIO)
- [livox_ros_driver2](https://github.com/linny-tech/livox_ros_driver2)
- [Mid360_imu_sim](https://github.com/linny-tech/Mid360_imu_sim)
- [offboard_hover](https://github.com/linny-tech/offboard_hover)
- [ego_gazebo_worlds](https://github.com/linny-tech/ego_gazebo_worlds)

FAST_LIO 包含 Git LFS 文件，单独克隆前请先执行 `git lfs install`。
