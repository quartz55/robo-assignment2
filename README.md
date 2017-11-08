# ROBO 2017/2018 Assignment 2

## Requirements
- [ROS Lunar Loggerhead](http://wiki.ros.org/lunar)
- [STDR Simulator](#installing_stdr)

## Compiling
1. Create a catkin workspace
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    ```

2. Install STDR simulator
    ```
    git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git src/stdr_simulator
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    catkin_make
    ```

3. Install source code
    ```
    mv <source_code_folder> src/assignment2
    catkin_make
    ```

## Running
1. Setup
    ```
    cd ~/catkin_ws
    source devel/setup.bash
    ```
    
2. Run one of the available examples
    - Launching Double D map with a placed robot
        ```
        roslaunch a2_launchers double_d_map_with_robot.launch
        ```
    
    - Launching Double D map with no robots (can be placed manually)
        ```
        roslaunch a2_launchers double_d_map.launch
        ```

## Extras
- To move a robot, right click on it on the map, choose `Move robot` and then click anywhere on the map you wish to place the robot
- To add a robot to an empty map, click the 2nd button to the left on the top toolbar, navigate to `<source_code>/a2_resources/robots` and choose on of the `.yaml` files