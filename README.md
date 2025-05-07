# Senior_Project_v2

4 Wheel autonomous navigation robot, deliverable for Senior Project of Robotics &amp; AI major


# Using Conda Environment

<details>
  <summary style="font-size:1.3em; font-weight:bold; cursor:pointer;">
    Auto Source ROS
  </summary>
    <div><br>

```bash
## Open (or create) the Conda activate.d script:
nano ~/miniforge3/envs/<YOUR_ENV_NAME>/etc/conda/activate.d/auto_source_ros.sh
```

```bash
#  Inside auto_source_ros.sh
## Source your ROS workspace
source /home/username/path/to/project/Senior_Project_v2/devel/setup.bash

## Append your build plugins to GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:/home/username/path/to/project/Senior_Project_v2/build/homewhere"
```

</details><br>

<details markdown="1">
  <summary style="font-size:1.4em; font-weight:bold; cursor:pointer;">
    Installing Packages
  </summary><br>

  <!-- Robostack -->
  <details markdown="1">
    <summary style="font-size:1.2em; font-weight:bold; cursor:pointer;">
      Robostack packages
    </summary>
    <div>
      Please follow the tutorial here:
      <a href="https://robostack.github.io/GettingStarted.html">RoboStack</a>
  </div>

```bash
mamba install ros-noetic-desktop-full compilers cmake pkg-config make ninja \
colcon-common-extensions catkin_tools rosdep
```

  </details><br>

  <!-- Turtlebot3 -->
  <details markdown="1">
    <summary style="font-size:1.2em; font-weight:bold; cursor:pointer;">
      Turtlebot3 packages
    </summary>

```bash
mamba install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs \
ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf \
ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* \
ros-noetic-rviz ros-noetic-gmapping ros-noetic-navigation \
ros-noetic-interactive-markers ros-noetic-dynamixel-sdk \
ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo
```
  </details><br>

  <!-- Other packages -->
  <details markdown="1">
    <summary style="font-size:1.2em; font-weight:bold; cursor:pointer;">
      Other packages
    </summary>

```bash
mamba install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
ros-noetic-global-planner ros-noetic-dwa-local-planner ros-noetic-teb-local-planner \
ros-noetic-costmap-converter ros-noetic-libg2o ros-noetic-mbf-costmap-core \
ros-noetic-mbf-abstract-core ros-noetic-mbf-msgs ros-noetic-mbf-utility
```

  </details><br>

  <!-- Fix Gazebo -->
  <details markdown="1">
    <summary style="font-size:1.2em; font-weight:bold; cursor:pointer;">
      Fix Gazebo
    </summary>

```bash
mamba install console_bridge cppzmq cryptography dartsim imath eigen freeglut \
gazebo=11.14.0 gstreamer gst-plugins-base libdrm libevent libfreetype \
libignition-cmake2 libignition-common3 libignition-fuel-tools4 \
libignition-math6 libignition-msgs5 libode ogre orocos-kdl pcl simbody \
tinyxml2 vtk
```

  </details>

</details><br>


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
- Navigation
- Multimap

World

- house
- turtle
- gas_station
- eng3
- 7hd
- 7hde



Run SLAM mode in turtle world

```bash
roslaunch homewhere main.launch mode:=slam world:=turtle
```

Run navigation mode in house world

```bash
roslaunch homewhere main.launch mode:=nav world:=house
```

Run default mode in 7hd room 3 world

```bash
roslaunch homewhere main.launch world:=7hd room:=3
```

Run multimap mode in 7hd world

```bash
roslaunch homewhere main.launch mode:=multi world:=7hd room:=1
```

# More Commands

Change configuration during runtime

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Open LiDAR in Rviz

```bash
roslaunch hls_lfcd_lds_driver view_hlds_laser.launch
```

Clear costmap

```bash
rosservice call /move_base/clear_costmaps "{}"
```

# Multimap commands

Publish goal at (9,0)

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 9.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

# Multimap Procedure

- Make all the maps separately using SLAM
- Set the origin of each map to a convenient location
- Set the center in map_table.yaml to the origin position in the global frame
- Set the polygon value using mode:=nav + green arrow to get the local coord of each vertex
- Set the orientation and entry/exit pose of each slope

