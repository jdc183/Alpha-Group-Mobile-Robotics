// Nicole Graf, Joseph Cressman, Andrew Capelli

// This node should contain a plan for a sequence of path 
// vertices, and send these as requests one at a timeto the
// des_state_publisher_service.  The coordinator will suspend
// while waiting for a response from the
// des_state_publisher_service.  If the response is “false”, 
// the coordinator should pause briefly, then re-send the 
// last, unsuccessful goal vertex.  (This could be adequate if
// a pedestrian blocks the robot, then subsequently walks away). 
// The navigation_coordinator will grow in sophistication to 
// incorporate path planning and path replanning 
// (e.g. to circumvent unexpected obstacles).

//dependencies
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <nav_msgs/Odometry.h>

//globals





//helper functions






//callbacks




int main(int argc, char **argv) {
    ros::init(argc, argv, "current_state_publisher"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    //ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("current_state", 1);
    //odom_publisher = pub; // let's make this global, so callback can use it

    // ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("odomCallback", 1);  
    // odomCallback = pub2;

    //ros::Subscriber odom_subscriber = nh.subscribe(/*mobot/odom*/"odom", 1, odomCallback); // edit for mobot odom
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}