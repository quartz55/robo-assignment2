# ROBO 2017/2018 Assignment 2

Basic wall following behavior for a STDR robot based on Subsumption Architecture

## Requirements

- [ROS Lunar Loggerhead](http://wiki.ros.org/lunar)
- [STDR Simulator](#compiling)

## Directory structure

```
catkin_ws  
│
└───src
    │
    └───assignment2
    │   │
    │   └───a2_follow_wall
    │   │   │
    │   │   │ ...
    │   │
    │   └───a2_resources
    │   │   │
    │   │   │ ...
    │   │
    │   └───a2_launchers
    │       │ 
    │       │ ...
    │
    └───stdr_simulator
        │
        │ ...
    
```

## Compiling

1. Create a catkin workspace
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    ```

2. Install STDR simulator
    ```bash
    git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git src/stdr_simulator
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    catkin_make
    ```

3. Install source code
    ```bash
    cp -r <source_code_folder>/assignment2 src/
    catkin_make
    ```

## Running

1. Setup
    ```bash
    cd ~/catkin_ws
    source devel/setup.bash
    ```

2. Run one of the available examples
    - Launching Double D map with a placed robot
        ```bash
        roslaunch a2_launchers double_d_map_with_robot.launch
        ```

    - Launching Double D map with no robots (can be [added manually](#extras))
        ```bash
        roslaunch a2_launchers double_d_map.launch
        ```

## Extras

- To move a robot, right click on it on the GUI, choose `Move robot` and then click anywhere on the map where you wish to place the robot
- To add a robot to an empty map, click the 2nd button to the left on the top toolbar, navigate to `<source_code>/assignment2/a2_resources/robots` and choose one of the available `.yaml` files