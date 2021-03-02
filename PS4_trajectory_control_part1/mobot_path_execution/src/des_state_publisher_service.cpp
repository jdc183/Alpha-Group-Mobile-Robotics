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

//dependencies
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <nav_msgs/Odometry.h>
#include <traj_builder/traj_builder.h> //Not sure if this is the right include
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// extra from newman
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>


//globals
ros::Publisher des_state_pub;
TrajBuilder trajBuilder;
nav_msgs::Odometry startState;
nav_msgs::Odometry endState;
geometry_msgs::PoseStamped startPose;
geometry_msgs::PoseStamped endPose;


//helper functions






//callbacks

//Service callback takes a Twist request and responds with a Bool of success or failure
//Not sure that Twist is the best idea for input
bool serviceCallback(geometry_msgs::Twist& request, std_msgs::Bool& response){
    //Set desired end pose
    endPose.pose.position.x = request.linear.x;
    endPose.pose.position.y = request.linear.y;
    endPose.pose.position.z = request.linear.z;
    endPose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(request.angular.z);
    
    //Set start pose - get from odom or tf somehow
    startPose.pose.position.x = //current x position;
    startPose.pose.position.y = //current y position;
    startPose.pose.position.z = //current z position;
    endPose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(//current orientation angle);
    
    std::vector<nav_msgs::Odometry> vec_of_states;
    nav_msgs::Odometry des_state;
    //build the trajectory - I think this is how to use trajBuilder but not sure
    //based on https://github.com/wsnewman/learning_ros_noetic/blob/main/Part_4/traj_builder/src/traj_builder_example_main.cpp 
    trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
    for (int i = 0; i < vec_of_states.size(); i++){
        des_state = vec_of_states[i];
        des_state.header.stamp = ros::Time::now();
        des_state_ub.publish(des_state);
        
        
        looprate.sleep();
    }
    return true;
}

// added from newman code (navigator) 
geometry_msgs::PoseStamped g_desired_pose;
int g_navigator_rtn_code;
void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const navigator::navigatorResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code=result->return_code;
    ROS_INFO("got object code response = %d; ",g_navigator_rtn_code);
    if (g_navigator_rtn_code==navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    }
    else if (g_navigator_rtn_code==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    }
    else {
        ROS_WARN("desired pose not reached!");
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher"); //name this node
    ros::NodeHandle nh; 
    ros::Publisher des_state_pub = nh.advertise<nav_msgs::Odometry>("des_state",1);
    ros::ServiceServer service = nh.advertiseService("trajectory_planner_service", serviceCallback);
    
    double dt = 0.01;
    ros::Rate looprate(1/dt);
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(1.0);
    
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}
