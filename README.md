# Project 5: Home Service Robot
### Directory Structure

    .myrobot_ws/ 
    └── src/
        └── my_robot/
            ├── config/
            │    ├── robot.rviz
            │    ├── base_local_planner_params.yaml
            │    ├── costmap_common_params.yaml
            │    ├── global_costmap_params.yaml
            │    └── local_costmap_params.yaml
            ├── images/
            │    ├── Home_Service_Robot.gif
            │    └── navigation_testing.png
            ├── launch/
            │    ├── amcl.launch
            │    ├── gazebo.launch
            │    ├── gmapping.launch
            │    └── world.launch
            ├── maps/
            │    ├── final_map.pgm
            │    └── final_map.yaml
            ├── materials/
            │    └── textures/
            │         └── realsense_diffuse.png
            ├── meshes/
            │    ├── hokuyo.dae
            │    └── realsense.dae
            ├── rviz_config/
            │    ├── gmapping.rviz
            │    ├── Home_Service_Robot.rviz
            │    └── navigation.rviz
            ├── shell/
            │    ├── gmapping_test.sh
            │    ├── Home_Service_Robot.sh
            │    └── navigation_test.sh
            ├── src/
            │    ├── object_marker.cpp
            │    └── pick_objects.cpp
            ├── urdf/
            │    ├── colors.xacro
            │    ├── my_robot.gazebo
            │    └── myrobot.urdf.xacro
            ├── worlds/
            │    └── arche.world
            ├── CMakeLists.txt
            ├── package.xml
            └── README.md


In this project, a mobile robot is asked to navigate to a pick-up zone for picking 
up a green cube. After that, it moves while carrying the cube to the drop-off
zone position and drops the cube there.

![Home Service Robot](/src/my_robot/images/Home_Service_Robot.gif)

## About the project
The mobile robot begins by driving around and scanning the area using a lidar to generate a static map of the environment. With this map in hand, it utilizes odometry and laser data to determine its position through adaptive Monte Carlo Localization (AMCL). When given a navigation goal, the robot plans its path using Dijkstra's algorithm and then navigates to the specified goal.

## Prerequisites
1. ROS (noetic), Gazebo on Linux
2. CMake & g++/gcc, C++11
3. Install xterm `sudo apt-get install xterm`
4. Install some dependencies

```
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-move-base
$ sudo apt-get install ros-noetic-slam-gmapping
```

## Build and Launch

1. Clone the project 
```
$ git clone https://github.com/Ahmed-Magdi1/Home-Service-Robot.git
```

2. Build the project
```
$ catkin_make
```

3. Run available scripts to launch
```
$ source devel/setup.bash
$ chmod u+x ./src/my_robot/shell/Home_Service_Robot.sh 
$ ./src/my_robot/shell/Home_Service_Robot.sh 
```

### Note: To regenerate new map, close all ROS terminals and run the gmapping_test.sh script again. When you are done, with all terminals open  execute

```rosrun map_server map_saver -f <map_name>```
## Then, replace:

### 1) In thw world launch file:

`<arg name="world" default="$(find my_robot)/worlds/<map_name>.world"/>` in the `launch/world.launch` with the newly created map `<map_name>`.

### 2) In the amcl launch file:

`<arg name="map_file" default="$(find my_robot)/map/<map_name>.yaml"/>` in the `launch/amcl.launch` with the newly created map `<map_name>`.

## Navigation Testing Instructions

To test the navigation functionality, follow these steps:

```
$ source devel/setup.bash
$ chmod u+x chmod u+x ./src/my_robot/shell/navigation_test.sh 
$ ./src/my_robot/shell/navigation_test.sh 
```
Once the script is running, send a `2D Nav Goal` in rviz to verify the navigation performance.

![Home Service Robot](/src/my_robot/images/navigation_testing.png)



