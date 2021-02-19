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

double speed = 1.0;
double heading;
double desired_heading = 0.0;
double_vec_srv::DblVecSrv srv;
ros::ServiceClient client;

ros::Publisher *g_twist_commander_ptr;
ros::Publisher twist_commander;
geometry_msgs::Twist twist_cmd;

bool front_obstruction = false;
bool left_obstruction = false;

tf::TransformListener *g_tfListener_ptr; 
tf::StampedTransform  g_robot_wrt_world_stf;

double heading_from_tf(tf::StampedTransform stf) {
    tf::Quaternion quat;
    quat = stf.getRotation();
    double theta = quat.getAngle();
    return theta;    
}

void front_obstruction_callback(const std_msgs::Bool& lidar_alarm_msg){
    front_obstruction = lidar_alarm_msg.data;
}

void left_obstruction_callback(const std_msgs::Bool& left_alarm_msg){
    left_obstruction = left_alarm_msg.data;
}

int main(int argc, char **argv) {
    ROS_INFO("Initializing...");
    ros::init(argc, argv, "stdr_commander"); // initialize a new node
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); // set name you use when you publish. declare the data type.
    g_twist_commander_ptr= &twist_commander; //synch twist commander and global twist commander
    client = n.serviceClient<double_vec_srv::DblVecSrv>("stdr_rotation_service"); //establish spin service client connection
    ros::Subscriber front_subscriber = n.subscribe("lidar_alarm", 1, front_obstruction_callback); //subscribe to front obstruct alarm
    ros::Subscriber left_subscriber = n.subscribe("left_lidar_alarm", 1, left_obstruction_callback); //subscribe to left wall tracker
    g_tfListener_ptr = new tf::TransformListener; //initialize transform listener

    //looping to wake up lookupTransform
    bool tferr=true;
    while (tferr) {
        tferr = false;
        try {            
            g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::spinOnce();
        }
    }
    ROS_INFO("Initialization complete.");

    ROS_INFO("Beginning navigation...");

    //set boolean to allow robot to move straight forward
    bool valid = false;

    //set boolean to determine whether or not the robot has reached its destination
    bool home = false;
    while(ros::ok() && !home){
        //navigation controller conditional statements

        if(!front_obstruction && left_obstruction && valid){
            //drive forward unitl interrupted
            twist_cmd.linear.x=speed;
            twist_cmd.linear.y=0.0;    
            twist_cmd.linear.z=0.0;
            twist_cmd.angular.x=0.0;
            twist_cmd.angular.y=0.0;
            twist_cmd.angular.z=0.0;

            g_twist_commander_ptr->publish(twist_cmd);
            twist_commander.publish(twist_cmd);
            ROS_INFO("Going straight.");
            valid = false;
        }
        else if(!left_obstruction){//If nothing is to our left
            g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);

            //extract the heading:
            heading= heading_from_tf(g_robot_wrt_world_stf);

            //update desired heading
            desired_heading = heading+PI/2;

            //Ask the spinner service to turn us left 90 degrees
            srv.request.vec_of_doubles.resize(1);
            srv.request.vec_of_doubles[0]=desired_heading;
            ROS_INFO("Calling turning service w request: %f",srv.request.vec_of_doubles[0]);
            if(client.call(srv)){
                ROS_INFO("Server call complete. Nothing was left! Turned left.");
            }

            //scoot forward until a wall is found
            twist_cmd.linear.x=speed;
            twist_cmd.linear.y=0.0;    
            twist_cmd.linear.z=0.0;
            twist_cmd.angular.x=0.0;
            twist_cmd.angular.y=0.0;
            twist_cmd.angular.z=0.0;
            g_twist_commander_ptr->publish(twist_cmd);
            twist_commander.publish(twist_cmd);

            ROS_INFO("Finding wall.");
            while(!left_obstruction){
                ros::spinOnce();
            }

            ROS_INFO("Wall found.");

            //stop robot now that wall is found;
            twist_cmd.linear.x=0.0;
            twist_cmd.linear.y=0.0;    
            twist_cmd.linear.z=0.0;
            twist_cmd.angular.x=0.0;
            twist_cmd.angular.y=0.0;
            twist_cmd.angular.z=0.0;
            g_twist_commander_ptr->publish(twist_cmd);
            twist_commander.publish(twist_cmd);

            valid = true;
        }
        else if(front_obstruction){//If something's in front of us...
            ROS_INFO("entered if(front_obstruction)");
            //Stop all motion:
            twist_cmd.linear.x=0.0;
            twist_cmd.linear.y=0.0;    
            twist_cmd.linear.z=0.0;
            twist_cmd.angular.x=0.0;
            twist_cmd.angular.y=0.0;
            twist_cmd.angular.z=0.0;
            g_twist_commander_ptr->publish(twist_cmd);  
            twist_commander.publish(twist_cmd);

            //Ask the spinner service to turn us right 90 degrees
            srv.request.vec_of_doubles.resize(1); 

            g_tfListener_ptr->lookupTransform("map", "robot0", ros::Time(0), g_robot_wrt_world_stf);

            //extract the heading:
            heading= heading_from_tf(g_robot_wrt_world_stf);

            //update desired heading
            if(fabs(heading)<0.005){
                desired_heading = heading+3*PI/2;
            }
            else{
                desired_heading = heading-PI/2;
            }
            srv.request.vec_of_doubles[0]=desired_heading;

            ROS_INFO("Calling service w request: %f",srv.request.vec_of_doubles[0]);
            if(client.call(srv)){
                ROS_INFO("Server called. Something was in front! Turned right.");
            }
            valid = true;
        }
        ros::spinOnce();
    }

    return 0;
}