//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <ps8_mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

using namespace std;

//globals
ros::ServiceClient client;
ros::NodeHandle n;
mobot_pub_des_state::path path_srv;
geometry_msgs::PoseStamped est_st_pose_base_wrt_map;
ros::ServiceServer service;

double distRobotFrontToCenter = 0.2;//This is a guess, we need to know how far the front of the robot is from its origin

//helper functions
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

//callbacks
bool backupCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response){
    response.success=false;
    ROS_WARN("Trigger received, moving backwards");
    ros::NodeHandle n2;
    
	mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here

    // we want to add to points. one intermediate to have the robot move left towards docking station and one just before the docking station. 
        // it should drive straight from the previous point of interest.
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // Center of robot aligned with center of dock
    pose.position.y = 0.412;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.x = 3.903 - distRobotFrontToCenter; // front of robot on center of dock
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
        
    client.call(path_srv);
	
    response.success=true;
    return response.success;
}

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "docking_service_1");
    ROS_WARN("initializing  ROS and node handle");

    ros::NodeHandle n;
    
    
    ROS_WARN("establishing servers");
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceServer service = n.advertiseService("dock_table1",callback);
	geometry_msgs::Quaternion quat;
	
	while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
	
    ros::spin();

    return 0;
}
