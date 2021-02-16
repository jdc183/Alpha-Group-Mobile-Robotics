# Alpha-Group-Mobile-Robotics
This is the upload for PS2: Wall following robot for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

This project consists of four nodes that work together in order to have the STDR follow along the left hand side of the robot. 

Node1:  wall_follower_lidar_alarm. This node searches directly in front of the robot to see if there is a wall in front of it. If there is, the robot will send a warning and the laser_alarm is set to true.

Node2:wall_follower_navigator

Node3:wall_follower_spinner

Node4:wall_follower_tracker
