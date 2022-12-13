# Robot Manager 
This package acts as a command interface to the robot model.  
The robot manager node will publish and record the heart beats (operaiton information) of the robot, 
including its current goal, current position, etc.  

The robot manager also provide commands interface such as set_routine service to command the robot 
to complete a set of waypoints called routine.


## Run
Launch robot manager
```
ros2 launch robot_manager robot_manager.launch.py
```

Call set_routine service  
The routine and waypoints are declared in the config/waypoints.yaml file.
```
ros2 service call /set_routine robot_manager_msgs/srv/SetRoutine "{routine: [B, A, C]}"
```
