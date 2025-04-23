# Senior_Project_v2
4 Wheel autonomous navigation robot, deliverable for Senior Project of Robotics &amp; AI major

# How to Build Project
```bash
catkin build
```

It will generate build devel log folder

```bash
source /Senior_Project_v2/devel/setup.bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/Senior_Project_v2/build/homewhere
```

# How to run

```bash
roslaunch homewhere main.launch
```

Mode
- Default
- SLAM

World
- house
- turtle
- gas_station


Run SLAM in turtle world

```bash
roslaunch homewhere main.launch mode:=slam world:=turtle
```

Run navigation in turtle world

```bash
roslaunch homewhere main.launch mode:=nav world:=turtle
```

# More Commands

Change coniguration during runtime

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Open LiDAR in Rviz

```bash
roslaunch hls_lfcd_lds_driver view_hlds_laser.launch
```

# Multimap commands

Terminal 1

```bash
roslaunch homewhere main.launch mode:=multi start_room:=1 world:=7hd
```

Terminal 2

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```