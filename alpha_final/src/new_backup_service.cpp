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

#include "pub_des_state_new.h"

#include <traj_builder/traj_builder.h> 
#include <mobot_pub_des_state/path.h>
#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>

//#include <alpha_final/BackupService.h>

//globals
double bkwd_dist_desired = 1; // 1m desired
double g_accel_max_ = -0.25;
double g_alpha_max_ = 0.2;
double g_speed_max_ = -0.25;
geometry_msgs::Twist g_halt_twist_;
double dt_ = 0.02;


double g_tolerance = 0.2;
double g_angle_tolerance = 0.1;

ros::ServiceClient pubdesClient;
mobot_pub_des_state::path path_srv;
geometry_msgs::PoseStamped est_st_pose_base_wrt_map;

std::vector<nav_msgs::Odometry> g_states;
geometry_msgs::PoseWithCovarianceStamped amcl_pose_data;
geometry_msgs::PoseStamped current;

//OdomTf odomTf;
XformUtils xform_utils;

ros::Publisher despub;
ros::Subscriber amcl_pose_sub;

double g_odom_tf_x;
double g_odom_tf_y;
double g_odom_tf_phi;

double r = 1; //backup distance

//helper functions
geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

bool pointToleranceCheck(double current, double end, double tolerance=g_tolerance){
    if((current<(end+tolerance)) && (current>(end-tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool angleToleranceCheck(double current, double end, double angle_tolerance=g_angle_tolerance){
    if((current<(end+angle_tolerance)) && (current>(end-angle_tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool poseToleranceCheck(geometry_msgs::Pose current, geometry_msgs::Pose end, double tolerance=g_tolerance, double angle_tolerance=g_angle_tolerance){
    double dist = sqrt(pow(current.position.x-end.position.x,2) + pow(current.position.y-end.position.y,2));
    ROS_INFO("Distance to goal: %f", dist);
    return dist < tolerance;
    // int score=0;
    // if(pointToleranceCheck(current.position.x, end.position.x)){
    //     score++;
    // }

    // if(pointToleranceCheck(current.position.y, end.position.y)){
    //     score++;
    // }

    // if(pointToleranceCheck(current.position.z, end.position.z)){
    //     //score++;
    // }

    // if(angleToleranceCheck(current.orientation.x, end.orientation.x)){
    //     //score++;
    // }

    // if(angleToleranceCheck(current.orientation.y, end.orientation.y)){
    //     //score++;
    // }

    // if(angleToleranceCheck(current.orientation.z, end.orientation.z)){
    //     score++;
    // }

    // if(angleToleranceCheck(current.orientation.w, end.orientation.w)){
    //     score++;
    // }

    // if(score==4){
    //     return true;
    // }
    // else{
    //     return false;
    // }
}

std::vector<nav_msgs::Odometry> build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
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

    return vec_of_states;
}




//callbacks
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped amclPose){
    ROS_WARN("ENTERED AMCL CALLBACK");
    amcl_pose_data = amclPose;
    current.pose = amclPose.pose.pose;
    current.header = amclPose.header;
}

bool backupCB(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response){
    response.success=false;
    ROS_WARN("Trigger received, moving backwards");
    ROS_WARN("Initializing for backup routine");
    ros::NodeHandle n2;
    OdomTf odomTf(&n2);
    ros::Rate looprate(1 / dt_);

    ROS_WARN("Calculating current position");
    g_odom_tf_x = current.pose.position.x;
    g_odom_tf_y = current.pose.position.y;
    g_odom_tf_phi = xform_utils.convertPlanarQuat2Phi(current.pose.orientation);
    /*
    est_st_pose_base_wrt_map = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    g_odom_tf_x = est_st_pose_base_wrt_map.pose.position.x;
    g_odom_tf_y = est_st_pose_base_wrt_map.pose.position.y;
    g_odom_tf_phi = xform_utils.convertPlanarQuat2Phi(est_st_pose_base_wrt_map.pose.orientation);
    */

    ROS_WARN("Setting goal position");
    geometry_msgs::PoseStamped goal = current;
    goal.pose.position.x = goal.pose.position.x - r * cos(g_odom_tf_phi);
    goal.pose.position.y = goal.pose.position.y - r * sin(g_odom_tf_phi);
    goal.pose.orientation = current.pose.orientation;
    /*
    geometry_msgs::PoseStamped goal = est_st_pose_base_wrt_map;
    goal.pose.position.x = goal.pose.position.x - r * cos(g_odom_tf_phi);
    goal.pose.position.y = goal.pose.position.y - r * sin(g_odom_tf_phi);
    goal.pose.orientation = est_st_pose_base_wrt_map.pose.orientation;
    */

    ROS_WARN("Building trajectory");
    std::vector<nav_msgs::Odometry> states;
    states = build_triangular_travel_traj(current, goal, states);
    //states = build_triangular_travel_traj(est_st_pose_base_wrt_map, goal, states);
    g_states = states;
    //DesStatePublisherNew dsp(n2);


    for(int i=0;i<states.size();i++){
        ROS_WARN("Publishing next state");
        despub.publish(states[i]);
        /*
        while(!poseToleranceCheck(current.pose,goal.pose,0.5,2*M_PI)){
            ROS_WARN("TRYING TO REVERSE");
            ros::spinOnce();
        }
        */
        //ros::Duration(0.01).sleep();
        looprate.sleep();
        ros::spinOnce();
    }

    response.success=true;
    return response.success;
}

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "new_backup_service");
    ROS_WARN("initializing ROS and node handle");

    ros::NodeHandle n;
    despub = n.advertise<nav_msgs::Odometry>("/desState", 1, true);
    amcl_pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,amclCallback);

    ROS_WARN("initializing OdomTF");
    OdomTf odomTf(&n);
    while (!odomTf.odom_tf_is_ready()) {
        ROS_WARN("waiting on odomTf warm-up");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_WARN("initializing XformUtils");
    XformUtils xform_utils;
    
    ROS_WARN("establishing servers");
    ros::ServiceClient pubdesClient = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service_backup");
    ros::ServiceServer service = n.advertiseService("new_backup",backupCB);

    ros::spin();

    return 0;
}
