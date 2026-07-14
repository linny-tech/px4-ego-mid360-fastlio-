# Fast-LIO to PX4 ENU bridge

This node corrects a Fast-LIO odometry stream before it is used by PX4 and
EGO-Planner. It publishes:

- corrected odometry on `/Odometry_enu`;
- corrected external vision on `/mavros/vision_pose/pose` at 30 Hz.

The default mapping is selected from the observed symptom that moving the
Gazebo aircraft along world X changes raw Fast-LIO Z:

```text
enu_x =  raw_z
enu_y = -raw_x
enu_z = -raw_y
```

This is the standard optical-frame to ROS ENU/FLU rotation. Confirm all three
axes before flight. If raw Fast-LIO already follows ENU, launch with
`axis_mode:=identity`. A custom right-handed 3x3 rotation can be supplied as
the private `axis_matrix` parameter.

Only this bridge may publish `/mavros/vision_pose/pose`; do not run the old
`camera_pose_node` at the same time.
