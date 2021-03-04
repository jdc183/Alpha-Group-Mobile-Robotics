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

// where we will put the intelligence. where path planning will go and reactions to things that went wrong. for now this is just a shell.
// watch out, positive spin requests may result in negative spins and vice versa. 
    // temporary fix, add a negative sign

//dependencies
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <traj_builder/traj_builder.h> 
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <dsp_service/DSPService.h>

//globals
ros::ServiceClient client;
TrajBuilder trajBuilder;

//helper functions
std::vector<geometry_msgs::PoseStamped> addPose(std::vector<geometry_msgs::PoseStamped> poseVec, double x, double y, double psi){
    ROS_INFO("entered addPose");
    ROS_INFO("creating new PoseStamped");
    geometry_msgs::PoseStamped newPose = trajBuilder.xyPsi2PoseStamped(x,y,psi);
    ROS_INFO("pushing onto poseVec");
    poseVec.push_back(newPose); 
    ROS_INFO("returning poseVec");
    return poseVec;
}

//main method
int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_coordinator"); //name this node
    ros::NodeHandle nh;
    client = nh.serviceClient<dsp_service::DSPService>("trajectory_planner_service"); //establish spin service client connection

    std::vector<geometry_msgs::PoseStamped> vec_of_poses;
    dsp_service::DSPService srv;
    geometry_msgs::PoseStamped next_goal_pose;

    /* Original poses:
    vec_of_poses = addPose(vec_of_poses,0,0,0);
    vec_of_poses = addPose(vec_of_poses,5,0,0);
    vec_of_poses = addPose(vec_of_poses,5,0,(-M_PI/2));
    vec_of_poses = addPose(vec_of_poses,5,-4,(-M_PI/2));
    vec_of_poses = addPose(vec_of_poses,5,-4,(-M_PI));
    vec_of_poses = addPose(vec_of_poses,-5,-4,(-M_PI));
    vec_of_poses = addPose(vec_of_poses,-5,-4,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-5,4,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-5,4,0);
    vec_of_poses = addPose(vec_of_poses,-1,4,0);
    vec_of_poses = addPose(vec_of_poses,-1,4,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-1,6,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-1,6,(M_PI));
    vec_of_poses = addPose(vec_of_poses,-8,6,(M_PI));
    vec_of_poses = addPose(vec_of_poses,-8,6,(-M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-8,-5,(-M_PI/2));
    */

    //Adjusted poses, w/ y-axis flip correction and wall tolerances:
    vec_of_poses = addPose(vec_of_poses,4.75,0,0);
    vec_of_poses = addPose(vec_of_poses,4.75,3.75,(-M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-4.75,3.75,(-M_PI));
    vec_of_poses = addPose(vec_of_poses,-4.75,-3.75,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-0.75,-3.75,0);
    vec_of_poses = addPose(vec_of_poses,-0.75,-6.25,(M_PI/2));
    vec_of_poses = addPose(vec_of_poses,-8,-6.25,(M_PI));
    vec_of_poses = addPose(vec_of_poses,-8,5,(-M_PI/2));


    for (int i = 0; i<vec_of_poses.size(); i++) {
        next_goal_pose = vec_of_poses[i];

        srv.request.end_pose = next_goal_pose;

        ROS_INFO("Requesting to move to (%f,%f)",next_goal_pose.pose.position.x,next_goal_pose.pose.position.y);
        while(!client.call(srv)) ROS_INFO("Failed to reach vertex (%f,%f) for some reason, trying again...",next_goal_pose.pose.position.x,next_goal_pose.pose.position.y);
    }

    //ros::Subscriber odom_subscriber = nh.subscribe(/*mobot/odom*/"odom", 1, odomCallback); // edit for mobot odom
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}