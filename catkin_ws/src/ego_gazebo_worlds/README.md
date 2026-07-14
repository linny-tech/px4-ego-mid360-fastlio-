# EGO-Planner Gazebo test worlds

ROS Noetic/Gazebo Classic 11 workspace for PX4 v1.13.3, MID360, Fast-LIO,
and EGO-Planner simulation.

The original `ego_50x50.world` has:

- a 50 m x 50 m floor and 4 m high boundary walls;
- an obstacle-free circular takeoff area at the map center;
- irregularly distributed boxes and cylinders;
- footprint dimensions/diameters between 1 m and 5 m;
- obstacle heights between 1 m and 4 m;
- only three large-footprint obstacles;
- a conservative obstacle-to-obstacle clearance greater than 1.5 m.

## Build in the unified workspace

This package is included under `catkin_ws/src`. On Ubuntu, run:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
chmod +x src/ego_gazebo_worlds/scripts/generate_ego_world.py
chmod +x src/ego_gazebo_worlds/scripts/generate_forest_world.py
catkin_make_isolated
source ~/catkin_ws/devel_isolated/setup.bash
```

## Regenerate and validate the fixed-seed map

The generated world and CSV layout are already included. To reproduce them:

```bash
cd ~/catkin_ws
python3 src/ego_gazebo_worlds/scripts/generate_ego_world.py
python3 src/ego_gazebo_worlds/scripts/generate_ego_world.py --validate-only
```

## Launch map only

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch ego_gazebo_worlds map_only.launch
```

## Launch PX4 + MAVROS + MID360 in this map

The `Mid360` and `iris_mid360` models must already exist under
`~/PX4-Autopilot/Tools/sitl_gazebo/models`.

```bash
source ~/.bashrc
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
```

The vehicle is spawned at map center `(x=0, y=0)` in the clear takeoff area.

## Forest map: 50 x 50 x 5 m

`ego_forest_50x50x5.world` is a deterministic forest scene for MID360 and
EGO-Planner testing. It contains 72 irregularly distributed tree trunks,
visual tree crowns, 5 m high boundaries, and clear pads on the west and east
edges. Tree trunks are the LiDAR/collision obstacles; crowns are visual only so
the flight corridor below the canopy remains usable.

The PX4/MID360 launch starts the vehicle on the west edge at
`(x=-21.0, y=0.0, z=0.2)`. The orange marker at `(21.0, 0.0)` is a suggested
opposite-edge EGO goal.

Regenerate and validate the fixed-seed forest map:

```bash
cd ~/catkin_ws
python3 src/ego_gazebo_worlds/scripts/generate_forest_world.py
python3 src/ego_gazebo_worlds/scripts/generate_forest_world.py --validate-only
```

Launch only the forest map:

```bash
roslaunch ego_gazebo_worlds forest_map_only.launch
```

Launch PX4, MAVROS, and MID360 at the west-edge spawn point:

```bash
roslaunch ego_gazebo_worlds px4_mid360_forest.launch
```
