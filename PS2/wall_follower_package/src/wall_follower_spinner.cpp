// Node designed to offer a service to rotate the robot by a desired angle

// We want to publish to the twist data structure like in PS1 or just directly to robot0/odom
// I think modeling it after HW1 would be easier for us though...

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
	
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); // initialize a new node
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds

    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0; // care about this for ps1
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0; // care about this for ps1

    ros::Rate loop_timer(1/sample_dt); // create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    // start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }
    // starting movements!
    twist_cmd.angular.z=yaw_rate; // start spinning in place approx 90 deg
    timer=0.0; //reset the timer
    while(timer<time_3_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    //done commanding the robot; node runs to completion
}