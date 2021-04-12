# PS8: Navigation Enhancements in the Lab Report
This is the upload for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

This project consists two packages that work together in order to have the mobot behave as though it is wall following in order to exit the starting pen. 

## Primary Package Functionality
The five scripts work as follows:

### new_nav_controller.cpp:
This controller utilizes the PID and commanded the robot to move to each desired point and called the necessary services to complete the desired tasks.

### new_backup_service.cpp: 
A service which intakes a desired position and uses the robot's current position and trajectory building to publish instructions intended to navigate the robot to its goal position in reverse.

## Operation
roslaunch alpha_final alpha_final_real.launch
rosrun alpha_final new_backup_service
rosrun alpha_final new_nav_controller

In order to run this on Gazebo, you will need the following commands:
roslaunch alpha_final alpha_final_sim_newmap.launch
rosrun alpha_final new_backup_service
rosrun alpha_final new_nav_controller

```

Alternatively, you can run each of the nodes, as well as the Gazebo simulator, from separate terminal tabs or windows. ```
```
