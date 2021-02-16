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
#define PI M_PI

using namespace std;

double speed = 1.0;

void front_obstruction_callback(const std_msgs::Bool& lidar_alarm_msg){
    bool front_obstruction = lidar_alarm_msg.data;
    if(front_obstruction){//If something's in front of us...
        //Stop and turn right 90 degrees
        //add code

    }
    else {
        //Go straight
        twist_cmd.linear.x=speed;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;
    }
}

void left_obstruction_callback(){

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); // initialize a new node
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.
    ros::Subscriber front_subscriber = nh.subscribe("lidar_alarm", 1, front_obstruction_callback);
    ros::Subscriber front_subscriber = nh.subscribe("left_lidar_alarm", 1, left_obstruction_callback);
    // while not triggering lidar alarm
    	// while not triggering wall follower navigator
    		// move forward via twist command stuff
    	// if triggeres wall follower navigator or lidar alarm
    		// run wall follower spinnner until signal goes away
    // that should be it?


    // from heading_test_client.cpp in order to talk to the service
    ros::init(argc, argv, "wall_follower_spinner");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<double_vec_srv::DblVecSrv>("stdr_rotation_service");
    double_vec_srv::DblVecSrv srv;
    double desired_heading;
    while(ros::ok()) {
        cout<<"enter a desired heading: ";
        cin>>desired_heading;
        srv.request.vec_of_doubles.resize(1);
        srv.request.vec_of_doubles[0]=desired_heading;
        //alt:
        //srv.request.vec_of_doubles.clear();
        //srv.request.vec_of_doubles.push_back(desired_heading);
        client.call(srv);  
    }
    return 0;
}