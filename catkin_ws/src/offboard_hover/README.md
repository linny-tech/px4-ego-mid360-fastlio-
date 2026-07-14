# Fast-LIO + PX4 OFFBOARD takeoff + EGO trajectory bridge

This ROS Noetic node uses Fast-LIO `/Odometry` as PX4 external vision and
implements this flight sequence:

```text
wait for PX4 and Fast-LIO
  -> ARM
  -> continuously publish the 1.5 m takeoff setpoint
  -> enter OFFBOARD
  -> take off and hold 1.5 m
  -> wait for EGO /planning/pos_cmd
  -> track EGO position and yaw commands
```

The node intentionally arms before it starts the OFFBOARD setpoint pre-stream.
It does not request OFFBOARD until the stream has been active for
`prestream_time` seconds.

## ROS interfaces

Inputs:

- `/Odometry` (`nav_msgs/Odometry`): raw Fast-LIO odometry.
- `/planning/pos_cmd` (`quadrotor_msgs/PositionCommand`): EGO trajectory-server
  output.
- `/mavros/state` (`mavros_msgs/State`).

Outputs:

- `/mavros/setpoint_position/local`: 30 Hz takeoff, hold, and EGO position/yaw
  setpoints.

By default PX4 external vision is supplied by the separate `camera_pose_node`
package used in `qiurongcan/px4_gazebo_EgoPlanner_sim`. Do not run two
publishers on `/mavros/vision_pose/pose`. Set `publish_vision_pose:=true` only
when the separate bridge is not running.

The first valid Fast-LIO sample defines the PX4 local origin. By default EGO
is expected to use the same raw `/Odometry` as its odometry input, so EGO world
commands are transformed into the same takeoff-relative frame before they are
sent to PX4. If EGO is already configured to publish takeoff-relative commands,
launch with `ego_command_is_relative:=true`.

Only EGO commands with `TRAJECTORY_STATUS_READY` are accepted. If EGO command
publication stops for longer than `ego_timeout`, the node holds the current
measured position and waits for a new trajectory. If Fast-LIO odometry becomes
stale, the node stops both vision and OFFBOARD setpoint streams so PX4's
configured failsafe can take control.

## PX4 v1.13.3 EKF2 configuration

Run in the PX4 SITL console, then restart PX4/Gazebo:

```text
param set EKF2_AID_MASK 24
param set EKF2_HGT_MODE 3
param set EKF2_EV_DELAY 0
param save
```

## Build

`quadrotor_msgs` is supplied by EGO-Planner and must be visible before this
package is compiled:

```bash
source /opt/ros/noetic/setup.bash

# Use this line when EGO-Planner is a separate workspace.
source /home/lin/ego-planner/devel/setup.bash

rospack find quadrotor_msgs

cp -a offboard_hover /home/lin/catkin_ws/src/
cd /home/lin/catkin_ws
catkin_make --pkg offboard_hover -j1
source /home/lin/catkin_ws/devel/setup.bash
```

If EGO packages already live in `/home/lin/catkin_ws/src`, the separate EGO
workspace source line is not needed.

## Run

Start PX4/Gazebo/MAVROS and Fast-LIO first. Verify:

```bash
rostopic hz /Odometry
rostopic type /planning/pos_cmd
```

Then run the controller:

```bash
roslaunch offboard_hover offboard_lidar_hover.launch
```

The vehicle arms, starts the setpoint stream, enters OFFBOARD, takes off to
1.5 m, stabilizes for two seconds, and then waits for EGO. Start EGO and provide
its goal through the configured waypoint interface. The EGO trajectory server
must publish `quadrotor_msgs/PositionCommand` on `/planning/pos_cmd`.

Useful overrides:

```bash
roslaunch offboard_hover offboard_lidar_hover.launch \
  odom_topic:=/Odometry \
  ego_cmd_topic:=/planning/pos_cmd \
  hover_height:=1.5 \
  ego_timeout:=0.5 \
  ego_command_is_relative:=false \
  publish_vision_pose:=false
```

Monitor the state machine and command stream:

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /mavros/setpoint_position/local
rostopic hz /planning/pos_cmd
```
