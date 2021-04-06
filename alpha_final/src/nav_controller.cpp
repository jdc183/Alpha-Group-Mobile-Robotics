//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//globals


//helper functions


//callbacks


//main function
int main(int argc, char **argv) {
	//ros init
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    
    //service instantiation


    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;


    return 0;
}