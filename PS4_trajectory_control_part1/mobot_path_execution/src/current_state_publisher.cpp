// Nicole Graf, Joseph Cressman, Andrew Capelli

// This node will later combine absolution pose information 
// (e.g. from GPS or LIDAR/map-based localization) with 
// high-speed Odom information.  For this assignment, this 
// node should merely subscribe to the Odom topic and 
// republish Odom on the topic “current_state”.  

#include <ros/ros.h> //Must include this for all ROS cpp projects

int main(int argc, char **argv) {
    ros::init(argc, argv, "current_state_publisher"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("current_state", 1);
    odom_publisher = pub; // let's make this global, so callback can use it
    // ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("odom_dist", 1);  
    // lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}