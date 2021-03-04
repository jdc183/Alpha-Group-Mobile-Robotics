// Nicole Graf, Joseph Cressman, Andrew Capelli

// This node should use functions from the traj_builder
// library (Part4/traj_builder) to construct triangular and 
// trapezoidal trajectory plans for either forward travel or 
// spin-in-place motions.  It should receive a goal pose as a 
// service request.  It should attempt to stream sequential 
// desired states in accordance with the request, resulting in
// returning either success or failure.  Reasons for failure 
// would include: encountering a lidar_alarm prior to reaching
// the goal pose, and failure to converge on the goal pose 
// within some tolerance.  In response to a lidar_alarm, this 
// service should dynamically construct and publish (stream) a
// graceful braking trajectory.

// The des_state_publisher should include a “mode” code for 
// spin-in-place, forward-travel, or halt (at specified final 
// state).

// throw away all queing. only accepts one point right now

//dependencies
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <traj_builder/traj_builder.h> 
#include <dsp_service/DSPService.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h> // boolean message 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// extra from newman
/*
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
*/

//globals
ros::Subscriber cur_state_sub;
ros::Subscriber lidar_alarm_sub;
ros::Publisher *g_des_state_pub_ptr;
TrajBuilder trajBuilder;
nav_msgs::Odometry startState;
nav_msgs::Odometry endState;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;
double dt = 0.01;
double tolerance = 0.2;
double angle_tolerance = 0.1;
bool alarmStatus = false;

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
        //score++;
    }

    if(angleToleranceCheck(current.orientation.w, end.orientation.w)){
        //score++;
    }

    if(score==2){
        return true;
    }
    else{
        return false;
    }
}

//callbacks
void currentStateCallback(const nav_msgs::Odometry current){
    g_start_pose.pose = current.pose.pose;
    g_start_pose.header = current.header;
}

void frontAlarmCallback(const std_msgs::Bool& newAlarmStatus){
    alarmStatus = newAlarmStatus.data;
}

bool serviceCallback(dsp_service::DSPServiceRequest& request, dsp_service::DSPServiceResponse& response){
    //Set desired end pose
    g_end_pose = request.end_pose;
    ROS_INFO("Request received: heading to (%f,%f)",g_end_pose.pose.position.x,g_end_pose.pose.position.y);

    ros::Rate looprate(1/dt);

    std::vector<nav_msgs::Odometry> l_vec_of_states;
    nav_msgs::Odometry des_state;
    //build the trajectory - I think this is how to use trajBuilder but not sure
    //based on https://github.com/wsnewman/learning_ros_noetic/blob/main/Part_4/traj_builder/src/traj_builder_example_main.cpp 
    trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, l_vec_of_states);
    for (int i = 0; i < l_vec_of_states.size(); i++){
        des_state = l_vec_of_states[i];
        des_state.header.stamp = ros::Time::now();
        g_des_state_pub_ptr->publish(des_state);
        
        
        looprate.sleep();
    }

    //need to add functionality to track lidar alarm
    if(poseToleranceCheck(g_start_pose.pose, g_end_pose.pose)){
        response.status = true;
    }
    else{
        response.status = false;
    }

    return response.status;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher"); //name this node
    ros::NodeHandle nh;

    ros::Rate looprate(1/dt);

    cur_state_sub = nh.subscribe<nav_msgs::Odometry>("current_state",1,currentStateCallback);
    lidar_alarm_sub = nh.subscribe("lidar_alarm",1,frontAlarmCallback);
    ros::Publisher des_state_pub = nh.advertise<nav_msgs::Odometry>("des_state",1);
    g_des_state_pub_ptr = &des_state_pub;
    ros::ServiceServer service = nh.advertiseService("trajectory_planner_service", serviceCallback);

    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(1.0);
    
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}