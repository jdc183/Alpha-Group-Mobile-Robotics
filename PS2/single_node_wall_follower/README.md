# Alpha-Group-Mobile-Robotics

In order to run this code, you will first need to boot up the stdr simulator: roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

UPDATING WITH ISSUES

Unfortunately, the Alpha Group experienced many issues with getting Node 4 to work correctly. We were able to successfully test nodes one through three. Because of this issue, we began to try to experiment with three different paths. One path is switching from transform to odom, but this method was looked at our last option because it would have required the most changes to our codes. Another method is debugging node 4 with the assistance of our TA and prof, but we were worried we may be off of our due time. Finally, we created one node that did the job of all four nodes. This node allows the robot to successfully navigate the course. To run this code, you need to have the STDR simulator open and type the following command:

rosrun single_node_wall_follower wall_follower.py


