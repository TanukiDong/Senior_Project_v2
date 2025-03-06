# Senior_Project_v2
4 Wheel autonomous navigation robot, deliverable for Senior Project of Robotics &amp; AI major

# How to Build Project

First, clone the repository and cd into it

```bash
git clone https://github.com/TanukiDong/Senior_Project_v2.git
cd Senior_Project_v2.git
```
Second, initialize and build catkin to generate build devel log folder

```bash
catkin init
catkin build
```

Third, source the project and export the plugin

```bash
pwd
# This cmd will show the full path of the cloned repo
# Should look sth like: /home/gems/Documents/senior/Senior_Project_v2/devel/setup.bash

source {PWD}/devel/setup.bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:{PWD}/build/homewhere
```

Since this step has to be run everytime a terminal is launched, you should add it to .bashrc

```bash
cd
nano .bashrc

# Add the following lines to the end of the file (Shown here is Gems' example)
# source /opt/ros/noetic/setup.bash # Source ROS
# source /home/gems/Documents/senior/Senior_Project_v2/devel/setup.bash 
# export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/gems/Documents/senior/Senior_Project_v2/build/homewhere
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