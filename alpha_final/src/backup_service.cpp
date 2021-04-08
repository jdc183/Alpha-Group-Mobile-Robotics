// Nicole Graf, Joseph Cressman, Andrew Capelli

// This code is a service to have the Jinx robot reverse 1 meter.

// i started by setting the accel_max and speed_max to be negative. I also edited down the entire pub_des_state file tto what I believe that we need. 
// To make sure that this code will run, I checked the traj_builder and noticed an issue with the triangular travel traj builder. I copied the portion over that we need and corrected these issues.
// I ~believe~ i have the correct components hardcoded to 1 meter (path_queue), but I am not sure becaause this code technically does not have a path queue
	// I made a set variable (bkwd_dist_desired) to accomplish this. I thought iit would be easier to change this amount at the top of our code if we want to change this value at a later point.

//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "pub_des_state.h" // i don't know if we want to use this
//#include "ps8_pub_des_state.h" // i don't know if we want to use this

//globals
double bkwd_dist_desired = 1.0; // 1m desired
double g_accel_max_;
double g_alpha_max_;
double g_speed_max_;
geometry_msgs::Twist g_halt_twist_;
double dt_;

geometry_msgs::PoseStamped g_start_pose;

geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

DesStatePublisher::DesStatePublisher(ros::NodeHandle& nh) : nh_(nh) {
    //as_(nh, "pub_des_state_server", boost::bind(&DesStatePublisher::executeCB, this, _1),false) {
    //as_.start(); //start the server running
    //configure the trajectory builder: 
    dt_ = dt; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
    trajBuilder_.set_dt(dt);
    //dynamic parameters: should be tuned for target system
    accel_max_ = -accel_max; //absolute accel_max?
    g_accel_max_=accel_max_;
    trajBuilder_.set_accel_max(accel_max_);
    alpha_max_ = alpha_max;
    g_alpha_max_ = alpha_max_;
    trajBuilder_.set_alpha_max(alpha_max_);
    speed_max_ = -speed_max;
    g_speed_max_ = speed_max_;
    trajBuilder_.set_speed_max(speed_max_);
    omega_max_ = 0;
    trajBuilder_.set_omega_max(omega_max_);
    path_move_tol_ = path_move_tol;
    trajBuilder_.set_path_move_tol_(path_move_tol_);
    initializePublishers();
    initializeServices();
    // set desired distance to be 1 meter
    
    //define a halt state; zero speed and spin, and fill with viable coords
    halt_twist_.linear.x = 0.0;
    halt_twist_.linear.y = 0.0;
    halt_twist_.linear.z = 0.0;
    halt_twist_.angular.x = 0.0;
    halt_twist_.angular.y = 0.0;
    halt_twist_.angular.z = 0.0;
    current_pose_ = trajBuilder_.xyPsi2PoseStamped(0,0,0);
    start_pose_ = current_pose_;
    end_pose_ = current_pose_;
    current_des_state_.pose.pose = current_pose_.pose;
    seg_start_state_ = current_des_state_;
    seg_end_state_ = current_des_state_;
    ROS_INFO("Finished constructing");
}

// from traj builder so we do not have to worry about issues with negative acceleration and speed from up top!
void build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_of_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    //des_state.twist.twist = halt_twist_; // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / (-g_accel_max_));
    int npts_ramp = round(t_ramp / dt_);
    double v_peak = g_accel_max_*t_ramp; // could consider special cases for reverse motion
    double d_vel = g_alpha_max_*dt_; // incremental velocity changes for ramp-up

    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = g_accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * g_accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * g_accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= g_accel_max_*dt_; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration        
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = g_halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}
// end traj build

//helper functions
//might not need 112-118
void DesStatePublisher::initializeServices() {
    ROS_INFO("Initializing Services");
    flush_path_queue_ = nh_.advertiseService("flush_path_queue_service",
            &DesStatePublisher::flushPathQueueCB, this);
    append_path_ = nh_.advertiseService("append_path_queue_service",
            &DesStatePublisher::appendPathQueueCB, this);
}

//member helper function to set up publishers;

void DesStatePublisher::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    desired_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desState", 1, true);
    des_psi_publisher_ = nh_.advertise<std_msgs::Float64>("/desPsi", 1);
}

//might not need 129-136
bool DesStatePublisher::flushPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_WARN("flushing path queue");
    while (!path_queue_.empty())
    {
        path_queue_.pop();
    }
    return true;
}

bool DesStatePublisher::appendPathQueueCB(mobot_pub_des_state::pathRequest& request, mobot_pub_des_state::pathResponse& response) {

    int npts = request.path.poses.size();
    ROS_INFO("appending path queue with %d points", npts);
    for (int i = 0; i < npts; i++) {
        path_queue_.push(request.path.poses[i]);
    }
    return true;
}

void DesStatePublisher::set_init_pose(double x, double y, double psi) {
    current_pose_ = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
}

//callbacks
// from lab 4 service
void currentStateCallback(const nav_msgs::Odometry current){
    g_start_pose.pose = current.pose.pose;
    g_start_pose.header = current.header;
}
// end inputs from lab 4

void DesStatePublisher::pub_next_state() {
    //state machine; results in publishing a new desired state
    //generate triangular trajectory and start publishing it with corresponding positions that go with it and clock that out
    //case PURSUING_SUBGOAL: //if have remaining pts in computed traj, send them
    //extract the i'th point of our plan:
    ROS_INFO("Entered next state function");

    current_des_state_ = des_state_vec_[traj_pt_i_];
    current_pose_.pose = current_des_state_.pose.pose;
    current_des_state_.header.stamp = ros::Time::now();
    ROS_INFO("Publishing current des state");
    desired_state_publisher_.publish(current_des_state_);
    ROS_INFO("Publishing complete");
    //next three lines just for convenience--convert to heading and publish
    // for rqt_plot visualization            
    des_psi_ = trajBuilder_.convertPlanarQuat2Psi(current_pose_.pose.orientation);
    float_msg_.data = des_psi_;
    des_psi_publisher_.publish(float_msg_); 
    traj_pt_i_++; // increment counter to prep for next point of plan
    //check if we have clocked out all of our planned states:
    if (traj_pt_i_ >= npts_traj_) {
        seg_end_state_ = des_state_vec_.back(); // last state of traj
        //path_queue_.pop(); 	// pop off destination vertex should be hardcoded feed it one meter
        //bkwd_dist_desired = path_queue_; // i think this may be correct?
        //end_pose_;
        ROS_INFO("reversed 1m: x = %f, y= %f",current_pose_.pose.position.x,
                current_pose_.pose.position.y);
    }
    //reak;
    /*
    default: //this should not happen
    ROS_WARN("motion mode not recognized!");
    desired_state_publisher_.publish(current_des_state_);
    break;
    */
}

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "backup_service");
    ros::NodeHandle nh; //instantiate a desired-state publisher object

    DesStatePublisher desStatePublisher(nh);
    //dt is set in header file pub_des_state.h
    ROS_INFO("establishing loop rate");    
    ros::Rate looprate(1 / dt_); //timer for fixed publication rate

    ROS_INFO("setting initial pose");
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here
    //desStatePublisher.append_path_queue(5.0,0.0,0.0);
    //desStatePublisher.append_path_queue(0.0,0.0,0.0);


    
    // main loop; publish a desired state every iteration
    ROS_INFO("Entering main while loop");
    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }

    return 0;
}