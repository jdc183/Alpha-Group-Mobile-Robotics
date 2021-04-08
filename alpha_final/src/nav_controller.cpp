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

#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>

#include <alpha_final/NavService.h>
#include <alpha_final/BackupService.h>


//globals

XformUtils xform_utils;

ros::ServiceClient navClient;
ros::ServiceClient backupClient;
ros::ServiceClient scanClient;
ros::ServiceClient grabClient;
ros::ServiceClient dropClient;
ros::ServiceClient pubdesClient;

std_srvs::Trigger trigger;

double distRobotFrontToCenter = 0.2;
double tolerance = 0.1;
double angle_tolerance = 0.05;

double goal1_x = 3.903;
double goal1_y = 0.412;
double goal1_phi = -0.012;

double goal2_x = 0.542;
double goal2_y = 2.572;
double goal2_phi = 1.539;
//callbacks



//helper functions
bool pointToleranceCheck(double current, double end){
    if((current<(end+tolerance)) && (current>(end-tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool angleToleranceCheck(double current, double end){
    if((current<(end+angle_tolerance)) && (current>(end-angle_tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool poseToleranceCheck(geometry_msgs::Pose current, geometry_msgs::Pose end){
    int score=0;
    if(pointToleranceCheck(current.position.x, end.position.x)){
        score++;
    }

    if(pointToleranceCheck(current.position.y, end.position.y)){
        score++;
    }

    if(pointToleranceCheck(current.position.z, end.position.z)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.x, end.orientation.x)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.y, end.orientation.y)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.z, end.orientation.z)){
        score++;
    }

    if(angleToleranceCheck(current.orientation.w, end.orientation.w)){
        score++;
    }

    if(score==4){
        return true;
    }
    else{
        return false;
    }
}

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

/*
void blockToArrival(double x, double y, double phi){
  geometry_msgs::Pose goal;
  goal.position.x=x;
  goal.position.y=y;
  goal.orientation = convertPlanarPhi2Quaternion(phi);

  geometry_msgs::Pose current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);

  bool arrived = false;
  while(!arrived){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    poseToleranceCheck(current, goal);
    ros::Duration(0.25).sleep();
  }
}
*/

void initializeServices(ros::NodeHandle n){
  ROS_WARN("initializing necessary services");
  ros::ServiceClient pubdesClient = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");

  /*
	navClient = n.serviceClient<alpha_final_test::NavServiceTest>("nav_p2p_service_test");
	while (!navClient.exists()) {
      ROS_INFO("waiting for nav service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected nav client to nav service");
  */

	backupClient = n.serviceClient<std_srvs::Trigger>("backup");
  while (!backupClient.exists()) {
    ROS_WARN("waiting for backup service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected backup client to backup service");

  /*
	//RESERVED FOR FUTURE USE

	scanClient = n.advertiseService("<scan service name>",scanCallback);
  while (!scanClient.exists()) {
    ROS_INFO("waiting for scan service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected scan client to scan service");

	grabClient = n.advertiseService("<grab service name>",grabCallback);
  while (!grabClient.exists()) {
    ROS_INFO("waiting for grab service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected grab client to grab service");

	dropClient = n.advertiseService("<drop service name>",dropCallback);
  while (!dropClient.exists()) {
    ROS_INFO("waiting for drop service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected drop client to drop service");

	*/
  ROS_WARN("service initialization complete");
}

//function to add 2 points to path queue for robot
//allows robot to align with goal point, then approach 
//xFirst boolean toggles whether robot travels in x or y direction first
mobot_pub_des_state::path addPointXYPtoPath(mobot_pub_des_state::path path_srv, double x, double y, double phi, bool xFirst){
  ROS_WARN("entered point-to-path function");
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";

  geometry_msgs::Quaternion quat;
  geometry_msgs::Pose pose;
  
  if(!xFirst){
    ROS_WARN("queueing points to y=%f, then x=%f",y,x);
    pose.position.x = 0.0; // Center of robot aligned with center of dock
    pose.position.y = y;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(phi);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.x = x - distRobotFrontToCenter; // front of robot on center of dock
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
  }
  else{
    ROS_WARN("queueing points to x=%f, then y=%f",x,y);
    pose.position.x = x; // Center of robot aligned with center of dock
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(phi);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.y = y - distRobotFrontToCenter; // front of robot on center of dock
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
  }
  
  ROS_WARN("Queue created, returning path");
  return path_srv;
}

//main function
int main(int argc, char **argv) {
	//ros init
  ROS_WARN("initializing  ROS and node handle");
  ros::init(argc, argv, "nav_controller");
  ros::NodeHandle n;
  ROS_WARN("initializing OdomTF");
  OdomTf odomTf(&n);
  ROS_WARN("initializing XformUtils");
  XformUtils xform_utils;
  
  //service initialization
  initializeServices(n);

  //defining goal paths
  ROS_WARN("creating goal paths");
  mobot_pub_des_state::path path_srv_goal1;
  path_srv_goal1 = addPointXYPtoPath(path_srv_goal1, goal1_x, goal1_y, goal1_phi, false);

  mobot_pub_des_state::path path_srv_goal2;
  path_srv_goal2 = addPointXYPtoPath(path_srv_goal2, goal2_x, goal2_y, goal2_phi, true);

  mobot_pub_des_state::path path_srv_end;
  path_srv_end = addPointXYPtoPath(path_srv_end, 0.0, 0.0, 0, false);

  ROS_WARN("issuing service call to reach station 1"); 
  pubdesClient.call(path_srv_goal1);

  geometry_msgs::Pose goal1;
  goal1.position.x=goal1_x;
  goal1.position.y=goal1_y;
  goal1.orientation = convertPlanarPhi2Quaternion(goal1_phi);

  geometry_msgs::PoseStamped current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);

  bool arrived = false;
  while(!arrived){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ROS_WARN("current position: %f, %f",current.pose.position.x,current.pose.position.y);
    ROS_WARN("goal position: %f, %f",goal1_x,goal1_y);
    arrived = poseToleranceCheck(current.pose, goal1);
    ros::Duration(0.25).sleep();
  }

  /*
  insert station 1 perception and manipulation here
  */

  ROS_WARN("issuing trigger to backup service");
  backupClient.call(trigger);

  ROS_WARN("issuing service call to reach station 2"); 
  pubdesClient.call(path_srv_goal2);

  /*
  insert station 2 perception and manipulation here
  */
  
  ROS_WARN("issuing trigger to backup service");
  backupClient.call(trigger);

  ROS_WARN("issuing service call to reach end position");
  pubdesClient.call(path_srv_end);

  return 0;
}