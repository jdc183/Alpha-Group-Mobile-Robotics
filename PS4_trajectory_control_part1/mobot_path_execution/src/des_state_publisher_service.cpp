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
#include <geometry_msgs/Stamped_pose.h>


//globals
ros::Publisher des_state_pub;




//helper functions






//callbacks
bool serviceCallback(double_vec_srv::DblVecSrvRequest& request, double_vec_srv::DblVecSrvResponse& response){
    
    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher"); //name this node
    ros::NodeHandle nh; 
    ros::Publisher des_state_pub = nh.advertise<nav_msgs::Odometry>("des_state",1);
    ros::ServiceServer service = nh.advertiseService("trajectory_planner_service", serviceCallback);
    
    double dt = 0.01;
    ros::Rate looprate(1/dt);
    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(1.0);
    
    // ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("odomCallback", 1);  
    // odomCallback = pub2;

    //ros::Subscriber odom_subscriber = nh.subscribe(/*mobot/odom*/"odom", 1, odomCallback); // edit for mobot odom
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}
