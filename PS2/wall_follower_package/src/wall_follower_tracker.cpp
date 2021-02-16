// This is our final node that will connect the other three written for PS2 to program the robot to follow the wall. 

//Default to moving forward with wall_follower_lidar_alarm and wall_follower_navigator running continously to check surroundings
	//Have robot move forward with twist vectors like PS1
	// Have if statements that are triggered by the statuses of the alarm and navigator programs
		// Reset the variables after execution of the processes

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>
#define PI = M_PI;
	
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); // initialize a new node
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.

    // while not triggering lidar alarm
    	// while not triggering wall follower navigator
    		// move forward via twist command stuff
    	// if triggeres wall follower navigator or lidar alarm
    		// run wall follower spinnner until signal goes away
    // that should be it?
