// Node designed to offer a service to rotate the robot by a desired angle

// We want to publish to the twist data structure like in PS1 or just directly to robot0/odom
// I think modeling it after HW1 would be easier for us though...

//heading service: receives requests for desired heading of stdr robot, and makes it so

// heading_service code from Newman. This node is done! Uses feedback and no open loop timing. Quite precise. Won't stop until it gets heading to 5 milliradians.
#include <ros/ros.h>
#include <double_vec_srv/DblVecSrv.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
//# tf...will get to this

using namespace std;
ros::Publisher *g_twist_commander_ptr;
tf::TransformListener *g_tfListener_ptr; 
tf::StampedTransform    g_robot_wrt_world_stf;    
double KV = 1.0;
double HEADING_TOL = 0.005; //this many radians error is considered within tolerance

double heading_from_tf(tf::StampedTransform stf) {
    tf::Quaternion quat;
    quat = stf.getRotation();
    double theta = quat.getAngle();
    return theta;    
}

bool callback(double_vec_srv::DblVecSrvRequest& request, double_vec_srv::DblVecSrvResponse& response)
{
    ROS_INFO("callback activated");
    double desired_heading = request.vec_of_doubles[0];
    ROS_INFO("received request to rotate to heading of %f",desired_heading);
    double heading;
    
    //perform action to carry out request
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;    
    g_tfListener_ptr->lookupTransform("map_static", "robot0", ros::Time(0), g_robot_wrt_world_stf);
    heading= heading_from_tf(g_robot_wrt_world_stf);
    double heading_err = 100.0; //not true; just init
   	while (fabs(heading_err)>HEADING_TOL) {          
        g_tfListener_ptr->lookupTransform("map_static", "robot0", ros::Time(0), g_robot_wrt_world_stf);

        //extract the heading:
        heading= heading_from_tf(g_robot_wrt_world_stf);
        ROS_INFO("current heading is %f",heading);
        heading_err = desired_heading-heading;
        if (heading_err>M_PI) { heading_err-=2*M_PI;}
        if (heading_err<-M_PI) {heading_err+=2*M_PI;}
        ROS_INFO("heading error = %f",heading_err);
    
        twist_cmd.angular.z=KV*heading_err;         
    
        g_twist_commander_ptr->publish(twist_cmd);
        ros::spinOnce();
        ros::Duration(0.01).sleep();    
  	}
    
    twist_cmd.angular.z=0.0;         //bring robot to a halt
    g_twist_commander_ptr->publish(twist_cmd); 
    
    ROS_INFO("converged with heading error = %f",heading_err);
    //as a courtesy, put resulting heading in the response
    response.vec_of_doubles.clear();
    response.vec_of_doubles.push_back(heading);     
    
  	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "heading_service");
	ros::NodeHandle n;
	ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	g_twist_commander_ptr= &twist_commander;
	ros::ServiceServer service = n.advertiseService("stdr_rotation_service", callback);
	g_tfListener_ptr = new tf::TransformListener; 
  
    ROS_INFO("trying to get robot pose w/rt world...");
    bool tferr=true;
    while (tferr) {
        tferr = false;
        try {            
            g_tfListener_ptr->lookupTransform("map_static", "robot0", ros::Time(0), g_robot_wrt_world_stf);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::spinOnce();
        }
    }   
  
	ros::spin();

	return 0;
}