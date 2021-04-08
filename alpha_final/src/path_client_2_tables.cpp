// Nicole Graf, Joseph Cressman, Andrew Capelli
// edited for a desired path to the docking station rather than a 1x1m or 3x3m square


//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service
// this one is a 3mx3m square path

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
using namespace std;

double distRobotFrontToCenter = 0.2;//This is a guess, we need to know how far the front of the robot is from its origin

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient backup_client = n.serviceClient<std_srvs::Trigger>("new_backup");
    geometry_msgs::Quaternion quat;
    std_srvs::Trigger trigger;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
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
    
    backup_client.call(trigger);
    
    pose.position.x = 0.542; // Center of robot aligned with center of dock
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.x = 2.572 - distRobotFrontToCenter; // front of robot on center of dock
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
        
    client.call(path_srv);

    return 0;
}
