// Nicole Graf, Joseph Cressman, Andrew Capelli
// Completed 2 March 2021?

// This node will be upgraded to perform “lane-drift” 
// correction, heading control and path progress control.  
// For this assignment, implement this equivalent to the 
// open-loop controller, which merely copies a desired state 
// twist to cmd_vel.  The controller should be modified to
// prepare for different control modes: spin-in-place, 
// straight-line-motion, and halt (to be implemented later).

//use open loop controller. print out a debug that says what mode it is in. wont effect the mode we are in at all
//dependencies
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//globals


//helper functions


ros::Publisher pub;
ros::Publisher *g_pub_ptr;
//callbacks
void des_state_callback(const nav_msgs::Odometry data){ 
    // might have to do something to get the actual twist data. pull from odom?
	g_pub_ptr->publish(data.twist.twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "modal_trajectory_controller"); //name this node
    ros::NodeHandle nh; 

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    g_pub_ptr = &pub;

    ros::Subscriber sub = nh.subscribe("des_state", 1, des_state_callback);

    ros::spin(); 

    return 0;
}