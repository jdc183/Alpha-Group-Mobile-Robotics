# Alpha-Group-Mobile-Robotics
This is the upload for PS2: Wall following robot for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

This project consists of four nodes that work together in order to have the STDR follow along the left hand side of the robot. 

Node1:  wall_follower_lidar_alarm. This node searches directly in front of the robot to see if there is a wall in front of it. If there is, the robot will send a warning and the laser_alarm is set to true.

Node2:  wall_follower_navigator. This node is designed to inspect a box on the left side of the robot. This is to detect that a wall is present on the left side of the robot. If no wall is detected, the robot will publish a warning and update the correct variables to notify the programs subscribing to it.

Node3:  wall_follower_spinner.  This node is designed to offer a service to rotate the robot by a desired angle. 

Node4:  wall_follower_tracker.  This is our final node that will connect the other three nodes written for PS2 to program the robot to follow the left hand wall of the course.

In order to run this code, you will first need to boot up the stdr simulator: roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

Afterwards, you will need to run: rosrun wall_following_robot wall_follower_tracker
