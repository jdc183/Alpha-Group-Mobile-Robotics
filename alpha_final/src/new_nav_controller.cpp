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
double angle_tolerance = 0.1;

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

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "append_path_client");
  ros::NodeHandle n;
  OdomTf odomTf(&n);
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

  geometry_msgs::PoseStamped goal = path_srv.request.path.poses[1];
  geometry_msgs::PoseStamped current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  }
  
  backup_client.call(trigger);
  goal = path_srv.request.path.poses[1];
  double phi = convertPlanarQuat2Phi(goal.pose.orientation);
  goal.pose.position.x = goal.pose.position.x - cos(phi);
  goal.pose.position.y = goal.pose.position.y - sin(phi);
  current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  }
  
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

  goal = path_srv.request.path.poses[1];
  current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  }

  backup_client.call(trigger);
  goal = path_srv.request.path.poses[1];
  phi = convertPlanarQuat2Phi(goal.pose.orientation);
  goal.pose.position.x = goal.pose.position.x - cos(phi);
  goal.pose.position.y = goal.pose.position.y - sin(phi);
  current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  }

  pose.position.x = 0.0; // Center of robot aligned with center of dock
  pose.position.y = 0.0;
  pose.position.z = 0.0; // let's hope so!
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.x = 0.0; // front of robot on center of dock
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);
      
  client.call(path_srv);

  goal = path_srv.request.path.poses[1];
  current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
    current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  }

  return 0;
}