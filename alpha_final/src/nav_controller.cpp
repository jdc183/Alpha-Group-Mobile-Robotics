//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mobot_pub_des_state/path.h>
#include <std_msgs/Float64.h>

#include <alpha_final/NavService.h>


//globals
ros::NodeHandle n;

ros::ServiceClient navClient;
ros::ServiceClient backClient;
ros::ServiceClient scanClient;
ros::ServiceClient grabClient;
ros::ServiceClient dropClient;

//callbacks

bool navP2PCallback(){

	return false;
}
/*
bool backupCallback(){
	
}

bool scanCallback(){
	
}

bool grabCallback(){
	
}

bool dropCallback(){
	
}
*/

//helper functions

void initializeServices(){
	navClient = n.serviceClient<alpha_final::NavService>("nav_p2p_service");
	while (!navClient.exists()) {
      ROS_INFO("waiting for nav service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected nav client to service");

    /*

	//backupClient = n.serviceClient("backup_service");
    while (!backupClient.exists()) {
      ROS_INFO("waiting for backup service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected backup client to service");

	//RESERVED FOR FUTURE USE

	scanClient = n.advertiseService("<scan service name>",scanCallback);
    while (!scanClient.exists()) {
      ROS_INFO("waiting for scan service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected scan client to service");

	grabClient = n.advertiseService("<grab service name>",grabCallback);
    while (!grabClient.exists()) {
      ROS_INFO("waiting for grab service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected grab client to service");

	dropClient = n.advertiseService("<drop service name>",dropCallback);
    while (!dropClient.exists()) {
      ROS_INFO("waiting for drop service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected drop client to service");

	*/
}

//main function
int main(int argc, char **argv) {
	//ros init
    ros::init(argc, argv, "nav_controller");
    ros::NodeHandle n;
    
    //service initialization
    initializeServices();

    geometry_msgs::Quaternion quat;
    


    //mobot_pub_des_state::path path_srv;


    return 0;
}