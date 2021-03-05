# Mobot Path Execution, Part 1--Alpha Group, Mobile Robotics
This is the upload for PS4: Mobot Path Execution Part 1 for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

This project consists two packages that work together in order to have the mobot behave as though it is wall following in order to exit the starting pen. 

##Primary Package Functionality
The five scripts work as follows:

###current_state_publisher.cpp:
Creates a node to publish the current pose of the mobot.

###des_state_publisher_service.cpp: 
A service which intakes a desired position and uses the robot's current position and trajectory building to publish instructions intended to navigate the robot to its goal position.

###lidar_alarm.cpp:
Receives input from the mobot's LIDAR sensor and monitors it, publishing a warning if the lidar detects something too close to the front of the mobot.

###modal_trajectory_controller.cpp: 
Receives the instructions from the desired-state publisher and sends them to the robot. Will later be updated to include lane-drift control, heading control, and path progress control.

###navigation_coordinator.cpp: 
Creates a vector of position vertices intended to act as a path for the mobot to reach its goal, then sequentially sends them to the desired-state publisher for it to develop instructions. 

##Included Service
This package includes an additional service package, named dsp_service. This package includes the server needed for the desired-state publisher, and must be included in the catkin_ws alongside

##Operation
A roslaunch file has been included, which is able to run all of the above nodes concurrently together with the Gazebo mobot simulator. To run this launch file: 
```
roslaunch mobot_path_execution mobot_path_executor.launch
```

Alternatively, you can run each of the nodes, as well as the Gazebo simulator, from six separate terminal tabs or windows. To do this, you will need to run the following commands, preferably in the following order:
```
roslaunch mobot_urdf mobot_in_pen.launch

rosrun mobot_path_execution current_state_publisher

rosrun mobot_path_execution lidar_alarm

rosrun mobot_path_execution modal_trajectory_controller

rosrun mobot_path_execution des_state_publisher_service

rosrun mobot_path_execution navigation_coordinator
```
