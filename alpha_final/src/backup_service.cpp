//include statements
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h> // boolean message 
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <math.h>

#include <traj_builder/traj_builder.h> 
#include <mobot_pub_des_state/path.h>
#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>

//#include <alpha_final/BackupService.h>

//globals
ros::ServiceClient pubdesClient;
mobot_pub_des_state::path path_srv;
geometry_msgs::PoseStamped est_st_pose_base_wrt_map;


//OdomTf odomTf;
XformUtils xform_utils;


double g_odom_tf_x;
double g_odom_tf_y;
double g_odom_tf_phi;

//helper functions


//callbacks
bool backupCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response){
    response.success=false;
    ROS_WARN("Trigger received, moving backwards");
    ros::NodeHandle n2;
    OdomTf odomTf(&n2);

    est_st_pose_base_wrt_map = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    g_odom_tf_x = est_st_pose_base_wrt_map.pose.position.x;
    g_odom_tf_y = est_st_pose_base_wrt_map.pose.position.y;
    g_odom_tf_phi = xform_utils.convertPlanarQuat2Phi(est_st_pose_base_wrt_map.pose.orientation);

    geometry_msgs::PoseStamped goal = est_st_pose_base_wrt_map;

    if(g_odom_tf_phi>(M_PI/4) && g_odom_tf_phi<(3*M_PI/4)){
        ROS_WARN("Instructing to back up in the +y direction...");
        goal.pose.position.y = goal.pose.position.y + 0.5;
    }
    else if(g_odom_tf_phi<(-M_PI/4) && g_odom_tf_phi>(-3*M_PI/4)){
        ROS_WARN("Instructing to back up in the -y direction...");
        goal.pose.position.y = goal.pose.position.y - 0.5;
    }
    else if(fabs(g_odom_tf_phi)<(M_PI/4)){
        ROS_WARN("Instructing to back up in the -x direction...");
        goal.pose.position.x = goal.pose.position.x - 0.5;
    }
    else if(fabs(g_odom_tf_phi)>(3*M_PI/4)){
        ROS_WARN("Instructing to back up in the +x direction...");
        goal.pose.position.x = goal.pose.position.x + 0.5;
    }

    path_srv.request.path.poses.push_back(goal);
    ROS_WARN("sending movement request");
    while(!pubdesClient.call(path_srv)){response.success=false;}

    response.success=true;
    return response.success;
}

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "backup_service");
    ROS_WARN("initializing  ROS and node handle");

    ros::NodeHandle n;
    ROS_WARN("initializing OdomTF");
    OdomTf odomTf(&n);
    ROS_WARN("initializing XformUtils");
    XformUtils xform_utils;
    
    ROS_WARN("establishing servers");
    ros::ServiceClient pubdesClient = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceServer service = n.advertiseService("backup",backupCB);

    ros::spin();

    return 0;
}