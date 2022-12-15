# Kick Warehouse Simulation
[![Build Status](https://github.com/sj0897/kickWarehouseSim/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/sj0897/kickWarehouseSim/actions/workflows/build_and_coveralls.yml)
[![Coverage Status](https://coveralls.io/repos/github/sj0897/kickWarehouseSim/badge.png?branch=master)]

Direct testing of algorithms in the real world has huge costs and safety concerns associated with them. This is where simulation comes into picture. The agenda of this component is to create a software to automate simulation for evaluation of various algorithms that will be running on a wheeled robot.

<p float="left">
  <img width="350" alt="robot_model" src="images/robot_model.png"> 
  <img width="300" alt="warehouse_gazebo" src="images/warehouse_gazebo.png"> 
  <img width="320" alt="warehouse_rviz" src="images/rviz_show_waypoints.png"> 
</p>


**Author:**  
|Name|UID|Github account|
|-----|-----|-----|
|Chang-Hong Chen|117397857|longhongc|
|Sparsh Jaiswal|117433968|sj0897| 
|Po-Yu Huang |117684681|danielforever| 

### Table of contents
- [**Dependencies**](#dependencies) 
- [**Build**](#build) 
- [**Run**](#run) 
  - [**Routine simulation**](#routine-simulation)
- [**Results**](#results)
- [**Design**](#design) 

## Dependencies
### Environment
- ROS2 foxy
- Ubuntu 20.04
### ROS2 Packages
These packages are in the third_party folder
- [Nav2](https://github.com/ros-planning/navigation2/tree/foxy-devel)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel)

## Build
Clone this repository to the src folder in ros2 workspace
```
cd {ros2 workspace}/src
git clone https://github.com/sj0897/kickWarehouseSim.git
```
The dependencies packages (Nav2 and slam toolbox) can be installed in two ways.
1. Clone git submodule
```
cd {ros2 workspace}/src/kickWarehouseSim
git submodule update --recursive
```
2. With apt-tool
```
sudo apt install ros-<distro>-slam-toolbox
sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup '~ros-<distro>-turtlebot3-.*'
```


Build the packages
```
cd {ros2 workspace}
colcon build
```

## Run 
### Routine simulation
Start warehouse gazebo simulation
```
ros2 launch warehouse_simulation warehouse_simulation.launch.py
```
Start robot model, navigation system, and mapping
```
ros2 launch robot_model warehouse_bot.launch.py 
```
Start RVIZ for visualization
```
ros2 launch warehouse_simulation rviz2.launch.py 
```
Start robot manager
```
ros2 launch robot_manager robot_manager.launch.py 
```
Send routine request through service  
The waypoints in the routine are defined in robot_manager/config/waypoints.yaml
```
ros2 service call /set_routine robot_manager_msgs/srv/SetRoutine "{routine: [A, B, C]}"
```
Routine config example  
<img width="300" alt="routine_config" src="images/parameter_configs.png"> 

## Results
Robot following a set of routine  
<img width="627" alt="routine_rviz" src="images/routine_rviz_speedup.gif"> 

## Design
### UML

<img width="627" alt="UML_Initial" src="UML/Initial/UML_Initial.png"> 

### ROS Node Graph

<img width="627" alt="ROS_NODE_GRAPH" src="UML/Initial/ROS_Node _Graph.png"> 

### Link to AIP Documents
https://docs.google.com/spreadsheets/d/1jPUXeID2PA99P3RgWnmy4Ot1bprAT-mLDxvRTo3mwnE/edit?usp=sharing
