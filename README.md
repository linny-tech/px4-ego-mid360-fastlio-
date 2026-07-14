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

## 独立功能包仓库

- [camera_pose_node](https://github.com/linny-tech/camera_pose_node)
- [ego-planner](https://github.com/linny-tech/ego-planner)
- [FAST_LIO](https://github.com/linny-tech/FAST_LIO)
- [livox_ros_driver2](https://github.com/linny-tech/livox_ros_driver2)
- [Mid360_imu_sim](https://github.com/linny-tech/Mid360_imu_sim)
- [offboard_hover](https://github.com/linny-tech/offboard_hover)
- [ego_gazebo_worlds](https://github.com/linny-tech/ego_gazebo_worlds)

FAST_LIO 包含 Git LFS 文件，单独克隆前请先执行 `git lfs install`。
