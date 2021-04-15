// steering_algorithm.h header file //
// wsn; Feb, 2016
// include this file in "lin_steering_wrt_odom.cpp"

#ifndef STEERING_ALGORITHM_H_
#define STEERING_ALGORITHM_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath> 

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

//#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

//Eigen is useful for linear algebra
//#include <Eigen/Eigen>
//#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <Eigen/LU>

// SERIOUSLY ADD INTEGRAL COMPONENTS LAST

const double UPDATE_RATE = 50.0; // choose the desired-state publication update rate
const double K_PHI= 1.0; // control gains for steering
const double K_PHI_D = 0.0; // NG control gain to dampen the spin controller. guessed the value. 
const double K_PHI_I = 0.0; // NG control gain to INTEGRATE the spin controller. guessed the value. 
    //CONTROLLER CONTINUES TO ACCUMULATE ERROR (int_heading_error) THE LONGER IT SITS THERE. IF MOTORS ARE OFF IT WILL ACCUMULATE TO INF AND ROBOT WILL LURCH. MAKE SURE TO FLUSH THESE TERMS BEFORE ROBOT IS TURNED ON!
    // NEVER LET THESE VALUES GET TOO LARGE; ANTIWINDUP. MAKE A SATURATION FUNCTION THAT LIMITS MAX. CAN USE EXACT SAME FUNCTION AS THE ONE THAT LIMITS ANG VEL PROVIDED IN CODE (LINE 243 ODOM)
    // HUNTING: RAMPS UP TORQUE UNTIL IT BREAKS FREE AND WILL OVERSHOOT BACK AND FORTH (WITH COULOMB FRICTION). DEAL WITH THIS BY MODIFYING IT TO ADD IN DEADBAN (IF STATEMENTS)

// USE CONTROL THEORY POLE PLACEMENT FOR VALUES OF K_PHIS. HAVE TO KNOW SOMETHING ABOUT THE INERTIA TO DO THIS
// CAN START AT 0 AND ADD P, D, AND I MANUALLY IF WE WANT TO AVOID THE DYNAMIC MODEL
// MODEL IN GAZEBO IS NOT ACCURATE AT ALL. IF WE TUNE IT WITH GAZEBO IT WILL BE WRONG
// TREAT ROBOT AS A UNIFORM MASS RECT PRISM THAT WEIGHS 300 LBS DIMENSIONS FROM GAZEBO AND FIND THE ROTATIONAL INERTIA OF THE RECT PILLAR 0.5MR^2
// M = 150 KG, R = 0.3 M. WHEN TUNING ROTATION ALL WE NEED IS THE ROTATIONAL INERTIAL.
// ALSO NEED THE EQUIVALENT MOTOR TORQUE PER A GIVEN COMMAND. DEPENDS ON MOTOR TORQUE, VELOCITY GAIN WE HAVE FROM OTHER CONTROLLER, AND GEAR RATIO. 
// TO FIND THIS MAGIC NUMBER, WE WILL NEED TO HAVE THE ROBOT MOVE WITH A SINUSOIDAL COMMAND FOR... CANT DO THIS IN LAB TIME

// GUESS AND CHECK WITH K_PHI THEN ADD K_PHI_D, THEN END WITH K_PHI_I
// RUN THROUGH PROJECT AND PROCESS WITH GAZEBO BUT KNOW THAT YOU WILL END UP WITH DIFFERENT GAINS FOR REAL ROBOT
const double K_DISP = 1.0; 
const double K_TRIP_DIST = 0.50;
const double K_TRIP_DIST_I = 0.0; // ADD SLOWLY AND BE CAREFUL OF: 
    //CONTROLLER CONTINUES TO ACCUMULATE ERROR (int_trip_dist_error) THE LONGER IT SITS THERE. IF MOTORS ARE OFF IT WILL ACCUMULATE TO INF AND ROBOT WILL LURCH. MAKE SURE TO FLUSH THESE TERMS BEFORE ROBOT IS TURNED ON!
    // NEVER LET THESE VALUES GET TOO LARGE; ANTIWINDUP. MAKE A SATURATION FUNCTION THAT LIMITS MAX. CAN USE EXACT SAME FUNCTION AS THE ONE THAT LIMITS ANG VEL PROVIDED IN CODE (LINE 243 ODOM)
    // HUNTING: RAMPS UP TORQUE UNTIL IT BREAKS FREE AND WILL OVERSHOOT BACK AND FORTH (WITH COULOMB FRICTION). DEAL WITH THIS BY MODIFYING IT TO ADD IN DEADBAN (IF STATEMENTS)

// dynamic limitations:  these apply to the steering controller; they may be larger than the limits on des state generation
const double MAX_SPEED = 0.50; // m/sec; adjust this
const double MAX_OMEGA = 0.50; //1.0; // rad/sec; adjust this

const int HALT = 0;
const int SPIN_IN_PLACE = 1;
const int LANE_DRIFT = 2;
const int REVERSE = 3;
const double dt = 0.1; //placeholder

// define a class, including a constructor, member variables and member functions
class SteeringController 
{
public:
    SteeringController(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    void lin_steering_algorithm(); // here is the heart of it...use odom state and desired state to compute twist command, and publish it
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);   
    double min_dang(double dang);  
    double sat(double x);
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber odom_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber des_state_subscriber_;
    
    ros::Publisher cmd_publisher_; // = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Publisher cmd_publisher2_; // = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);
    ros::Publisher steering_errs_publisher_;
    
    tf::TransformListener* tfListener_;
    tf::StampedTransform mapToOdom_; 
    tf::StampedTransform baseLink_wrt_map_;    
    tf::StampedTransform odomToMap_;
    geometry_msgs::Twist twist_cmd_;
    geometry_msgs::TwistStamped twist_cmd2_;    
    double current_speed_des_;
    double current_omega_des_;

    int mode;
    

    //state values from odometry; these will get filled in by odom callback
    nav_msgs::Odometry current_odom_; // fill in these objects from callbacks
    geometry_msgs::Pose odom_pose_;    
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_phi_;
    geometry_msgs::Quaternion odom_quat_; 
    //Eigen::Vector2d odom_xy_vec_;
    
    //state values from desired state; these will get filled in by desStateCallback
    nav_msgs::Odometry des_state_; 
    geometry_msgs::Pose des_state_pose_;    
    double des_state_vel_;
    double des_state_omega_;
    double des_state_x_;
    double des_state_y_;
    double des_state_phi_;
    geometry_msgs::Quaternion des_state_quat_;  
    //Eigen::Vector2d des_xy_vec_;    
    
    // message to hold/publish steering performance data
    std_msgs::Float32MultiArray steering_errs_;
        
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
 
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);    
        
}; 

#endif  
