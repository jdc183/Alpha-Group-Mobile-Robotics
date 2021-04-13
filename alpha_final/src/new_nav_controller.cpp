//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mobot_pub_des_state/path.h>
#include <std_msgs/Float64.h>
#include <math.h>

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

ros::Subscriber amcl_pose_sub;

std_srvs::Trigger trigger;

geometry_msgs::PoseStamped current;
geometry_msgs::PoseWithCovarianceStamped amcl_pose_data;

double distRobotFrontToCenter = 0.3;//0.2;
double tolerance = 3;
double angle_tolerance = 2*M_PI;

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
    double dist = sqrt(pow(current.position.x-end.position.x,2) + pow(current.position.y-end.position.y,2));
    ROS_INFO("Distance to goal: %f", dist);
    return dist < tolerance;
    /*int score=0;
    if(pointToleranceCheck(current.position.x, end.position.x)){
        score++;
        // ROS_INFO("X good");
    }

    if(pointToleranceCheck(current.position.y, end.position.y)){
        score++;
        // ROS_INFO("Y good");
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
        //ROS_INFO("Quat z good");
    }

    if(angleToleranceCheck(current.orientation.w, end.orientation.w)){
        score++;
        //ROS_INFO("Quat w good");
    }

    if(score==4){
        return true;
    }
    else{
        return false;
    }
    */
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

//callback
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped amclPose){
	ROS_WARN("ENTERED AMCL CALLBACK");
	amcl_pose_data = amclPose;
	current.pose = amclPose.pose.pose;
	current.header = amclPose.header;
}

//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "append_path_client");
  ros::NodeHandle n;
  OdomTf odomTf(&n);
  ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
  ros::ServiceClient backup_client = n.serviceClient<std_srvs::Trigger>("new_backup");
  amcl_pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,amclCallback);
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
  //geometry_msgs::PoseStamped current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH FIRST GOAL");
    ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  ros::Duration(5).sleep();
  ros::spinOnce();

  ROS_WARN("About to back it up");
  backup_client.call(trigger);
  ROS_WARN("Currently backing it up");
  goal = path_srv.request.path.poses[1];
  double phi = convertPlanarQuat2Phi(current.pose.orientation);
  goal.pose.position.x = current.pose.position.x - cos(phi);
  goal.pose.position.y = current.pose.position.y - sin(phi);
  goal.pose.orientation = current.pose.orientation;
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO BACK UP AFTER FIRST GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);

    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  ros::Duration(5).sleep();
  ros::spinOnce();
  
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

  goal = path_srv.request.path.poses[3];
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH SECOND GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  ros::Duration(5).sleep();
  ros::spinOnce();

  backup_client.call(trigger);
  goal = path_srv.request.path.poses[3];
  phi = convertPlanarQuat2Phi(current.pose.orientation);
  goal.pose.position.x = current.pose.position.x - cos(phi);
  goal.pose.position.y = current.pose.position.y - sin(phi);
  goal.pose.orientation = current.pose.orientation;
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO BACK UP AFTER SECOND GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  ros::Duration(5).sleep();
  ros::spinOnce();

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

  goal = path_srv.request.path.poses[5];
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH END GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  return 0;
}
