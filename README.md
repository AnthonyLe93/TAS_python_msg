# ROS package to interface ati f/t sensor with UR10 arm via Labjack DAQ U6
## Dependencies required:
#### [ROS noetic](http://wiki.ros.org/noetic)
#### [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/README.md)
#### [ur_robot_driver](https://github.com/AnthonyLe93/ur_robot_driver)
#### [Labjack Exodriver](https://labjack.com/support/software/installers/exodriver)
#### Labjack Python package
```bash
pip install LabJackPython
```
### Description:
This repository is used to collect data from a mini45 ati f/t sensor via a Labjack U6 daq. The sensor data is stream via 
USB interface which provides flexibility to integrate other custom sensors.
This repo works in conjunction with the modified __ur_robot_driver__ above to work with the current robot arm setup.
Please replace the preinstalled __ur_robot_driver__ package from __Universal_Robots_ROS_Driver__ with the modified version above.

### Building 
```bash
# Install ROS noetic
http://wiki.ros.org/noetic/Installation/Ubuntu

# source global ros
$ source /opt/ros/noetic/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

