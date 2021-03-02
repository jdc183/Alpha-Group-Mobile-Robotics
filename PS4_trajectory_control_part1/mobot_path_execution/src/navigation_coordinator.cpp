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
// extra from newman
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

// NEWMAN CODE:
// example_navigator_action_client: 
// wsn, April, 2016
// illustrates use of navigator action server called "navigatorActionServer"
int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    actionlib::SimpleActionClient<navigator::navigatorAction> navigator_ac("navigatorActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to navigator action server"); // if here, then we connected to the server; 
     
    navigator::navigatorGoal navigation_goal;
    
    navigation_goal.location_code=navigator::navigatorGoal::HOME;
    
    ROS_INFO("sending goal: ");
        navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb); // we could also name additional callback functions here, if desired

        
        bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    return 0;
}