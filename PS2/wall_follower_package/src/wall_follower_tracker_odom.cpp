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
#include <double_vec_srv/DblVecSrv.h>
#include <tf/transform_listener.h>
#define PI M_PI

using namespace std;

// Ian Help Changes
double heading_from_odom(nav_msgs::Odometry odom){
    tf::Quaternion quat;
    quat = odom.pose.pose.orientation;
    double theta = quat.getAngle()

}

void odom_callback(nav_msgs::Odometry odom){

	heading = heading_from_odom(odom);

}

        //double speed = 1.0; // in line 59
// end Ian help

double speed = 1.0;
double heading;
double desired_heading = 0.0;
double_vec_srv::DblVecSrv srv; // confused on this line.... it is not blue because it is attached to the var nam... do we need another double?
ros::ServiceClient client;

ros::Publisher twist_commander;
geometry_msgs::Twist twist_cmd;

bool front_obstruction = false;
bool left_obstruction = false;

tf::TransformListener *g_tfListener_ptr; 
tf::StampedTransform  g_robot_wrt_world_stf;

// I dont think we want to do this next section
/*
double heading_from_tf(tf::StampedTransform stf) {
    tf::Quaternion quat;
    quat = stf.getRotation();
    double theta = quat.getAngle();
    return theta;    
}
*/

void front_obstruction_callback(const std_msgs::Bool& lidar_alarm_msg){
    front_obstruction = lidar_alarm_msg.data;
    /*
    if(front_obstruction){//If something's in front of us...

        //Stop and turn right 90 degrees:
        twist_cmd.linear.x=0.0;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;
        twist_commander.publish(twist_cmd);

        desired_heading = 0.0;
        srv.request.vec_of_doubles.resize(1);

        g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);

        //extract the heading:
        heading= heading_from_tf(g_robot_wrt_world_stf);

        //update desired heading
        desired_heading = heading-PI/2;

        srv.request.vec_of_doubles[0]=desired_heading;
        client.call(srv);
    }
    else if(!left_obstruction){//If nothing is to our left
        //turn left:

        g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);

        //extract the heading:
        heading= heading_from_tf(g_robot_wrt_world_stf);

        //update desired heading
        desired_heading = heading+PI/2;

        //Ask the spinner service to turn us left 90 degrees
        srv.request.vec_of_doubles.resize(1);
        srv.request.vec_of_doubles[0]=desired_heading;
        client.call(srv);
    }
    else {
        //Go straight
        twist_cmd.linear.x=speed;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;
        twist_commander.publish(twist_cmd);

    }*/
}

void left_obstruction_callback(const std_msgs::Bool& left_alarm_msg){
    left_obstruction = left_alarm_msg.data;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); // initialize a new node
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.
    ros::Subscriber front_subscriber = n.subscribe("lidar_alarm", 1, front_obstruction_callback);
    ros::Subscriber left_subscriber = n.subscribe("left_lidar_alarm", 1, left_obstruction_callback);

    // Ian help changes
    twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.
    ros::Subscriber front_subscriber = n.subscribe("/lidar_alarm", 1, front_obstruction_callback);

    ros::Subscriber lodom_sub = n.subscribe("/robot0/odom", 1, odom_callback);

	while(ros::ok()) {

		if(lidar_alarm==false){
			geometry_msgs::Twist twist_cmd;
			twist_cmd.linear.x=speed;
        	twist_cmd.linear.y=0.0;    
	        twist_cmd.linear.z=0.0;
	        twist_cmd.angular.x=0.0;
	        twist_cmd.angular.y=0.0;
	        twist_cmd.angular.z=0.0;
	        twist_commander.publish(twist_cmd);
		}

		ros::spinOnce();

	}
// end Ian help changes


    // while not triggering lidar alarm
    	// while not triggering wall follower navigator
    		// move forward via twist command stuff
    	// if triggeres wall follower navigator or lidar alarm
    		// run wall follower spinnner until signal goes away
    // that should be it?


/*    // from heading_test_client.cpp in order to talk to the service
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
*/
    ROS_INFO("entered conditional");
    if(front_obstruction){//If something's in front of us...
        ROS_INFO("entered if(front_obstruction)");
        //Stop and turn right 90 degrees:
        twist_cmd.linear.x=0.0;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;
        twist_commander.publish(twist_cmd);

        desired_heading = 0.0;
        srv.request.vec_of_doubles.resize(1);

        ROS_INFO("before");
        g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);
        ROS_INFO("after");

        //extract the heading:
        //heading= heading_from_tf(g_robot_wrt_world_stf); // we want odom
        void left_obstruction_callback(const std_msgs::Bool& left_alarm_msg){

    	left_obstruction = left_alarm_msg.data;

		}
// i think lines 207-214 should be inside the void?
        //update desired heading
        desired_heading = heading-PI/2;

        srv.request.vec_of_doubles[0]=desired_heading;
        client.call(srv);
        ROS_INFO("Something in front! Turning right.");
    }
    else if(left_obstruction){//If nothing is to our left
        ROS_INFO("else if(!left_obstruction)");
        //turn left:

        g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);
        ROS_INFO("declared TFListener");

        //extract the heading:
        heading= heading_from_tf(g_robot_wrt_world_stf);
        ROS_INFO("heading from tf extracted");

        //update desired heading
        desired_heading = heading+PI/2;
        ROS_INFO("heading from tf updated 90degrees");

        //Ask the spinner service to turn us left 90 degrees
        srv.request.vec_of_doubles.resize(1);
        ROS_INFO("resized successfully");
        srv.request.vec_of_doubles[0]=desired_heading;
        ROS_INFO("set first index of vector");
        client.call(srv);
        ROS_INFO("Server called. Nothing left! Turning left.");
    }
    else {
        ROS_INFO("else");
        //Go straight
        twist_cmd.linear.x=speed;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;
        twist_commander.publish(twist_cmd);
        ROS_INFO("Going straight.");
    }


    g_tfListener_ptr = new tf::TransformListener; 
  
    ROS_INFO("trying to get robot pose w/rt world...");
    bool tferr=true;
    /*
    while (tferr) {
        tferr = false;
        try {            
            g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::spinOnce();
        }
    }   */
    // ros::spin(); // this is the spin issue
    return 0;
}