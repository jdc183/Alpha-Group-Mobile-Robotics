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
double bkwd_dist_desired = 1.0; // 1m desired
double g_accel_max_ = -0.5;
double g_alpha_max_ = 0.2;
double g_speed_max_ = -1.0;
geometry_msgs::Twist g_halt_twist_;
double dt_ = 0.02;

ros::ServiceClient pubdesClient;
mobot_pub_des_state::path path_srv;
geometry_msgs::PoseStamped est_st_pose_base_wrt_map;


//OdomTf odomTf;
XformUtils xform_utils;


double g_odom_tf_x;
double g_odom_tf_y;
double g_odom_tf_phi;

double r = 1.0;

//helper functions
geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
    geometry_msgs::PoseStamped end_pose,
    std::vector<nav_msgs::Odometry> &vec_of_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    //des_state.twist.twist = halt_twist_; // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / (-g_accel_max_));
    int npts_ramp = round(t_ramp / dt_);
    double v_peak = g_accel_max_*t_ramp; // could consider special cases for reverse motion
    double d_vel = g_alpha_max_*dt_; // incremental velocity changes for ramp-up

    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = g_accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * g_accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * g_accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= g_accel_max_*dt_; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration        
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = g_halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

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
    goal.pose.position.x = goal.pose.position.x - r * cos(g_odom_tf_phi);
    goal.pose.position.y = goal.pose.position.y - r * sin(g_odom_tf_phi);
    goal.pose.orientation = est_st_pose_base_wrt_map.pose.orientation;

    /*
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
    */

    //response.success=true;
    return response.success;
}

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "backup_service_test");
    ROS_WARN("initializing  ROS and node handle");

    ros::NodeHandle n;
    ROS_WARN("initializing OdomTF");
    OdomTf odomTf(&n);
    ROS_WARN("initializing XformUtils");
    XformUtils xform_utils;
    
    ROS_WARN("establishing servers");
    ros::ServiceClient pubdesClient = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceServer service = n.advertiseService("backup_test",backupCB);

    ros::spin();

    return 0;
}