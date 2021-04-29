//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mobot_pub_des_state/path.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <exception>

#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>

#include <alpha_final/NavService.h>
#include <alpha_final/BackupService.h>

//pcl includes:
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/PCLHeader.h>

#include <pcl_utils/pcl_utils.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
int g_ans;



//globals

XformUtils xform_utils;

ros::ServiceClient navClient;
ros::ServiceClient backupClient;
ros::ServiceClient scanClient;
ros::ServiceClient grabClient;
ros::ServiceClient dropClient;
ros::ServiceClient pubdesClient;

ros::Subscriber amcl_pose_sub;

std_srvs::Trigger trigger;

geometry_msgs::PoseStamped current;
geometry_msgs::PoseWithCovarianceStamped amcl_pose_data;

double distRobotFrontToCenter = 0.3;//0.2;
double tolerance = .2;
double angle_tolerance = 2*M_PI;

double goal1_x = 3.903;
double goal1_y = 0.412;
double goal1_phi = -0.012;

double goal2_x = 0.542;
double goal2_y = 2.572;
double goal2_phi = 1.539;
//callbacks

bool got_kinect_image = false; //snapshot indicator
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_kinect_image) { // once only, to keep the data stable
        ROS_INFO("got new selected kinect image");
        pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
        ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
        got_kinect_image = true;
    }
}



//helper functions
bool pointToleranceCheck(double current, double end){
    if((current<(end+tolerance)) && (current>(end-tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool angleToleranceCheck(double current, double end){
    if((current<(end+angle_tolerance)) && (current>(end-angle_tolerance))){
        return true;
    }
    else{
        return false;
    }
}

bool poseToleranceCheck(geometry_msgs::Pose current, geometry_msgs::Pose end){
    double dist = sqrt(pow(current.position.x-end.position.x,2) + pow(current.position.y-end.position.y,2));
    ROS_INFO("Distance to goal: %f", dist);
    return dist < tolerance;
    /*int score=0;
    if(pointToleranceCheck(current.position.x, end.position.x)){
        score++;
        // ROS_INFO("X good");
    }

    if(pointToleranceCheck(current.position.y, end.position.y)){
        score++;
        // ROS_INFO("Y good");
    }

    if(pointToleranceCheck(current.position.z, end.position.z)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.x, end.orientation.x)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.y, end.orientation.y)){
        //score++;
    }

    if(angleToleranceCheck(current.orientation.z, end.orientation.z)){
        score++;
        //ROS_INFO("Quat z good");
    }

    if(angleToleranceCheck(current.orientation.w, end.orientation.w)){
        score++;
        //ROS_INFO("Quat w good");
    }

    if(score==4){
        return true;
    }
    else{
        return false;
    }
    */
}

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//Find table frame functions
double plane_z_interp(float x, float y, float plane_dist, Eigen::Vector3f normal) {
    double z;
    z = (plane_dist - x * normal(0) - y * normal(1)) / normal(2);
    return z;
}

double createInterpolatedPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f plane_normal, double plane_dist, double x_max, double y_max, double x_min, double y_min) {
    //generate points in the corresponding plane:
    //double plane_z_interp(double x, double y, double plane_dist, Eigen::Vector3d normal)

    input_cloud_ptr->points.clear();
    double zPlaneSum = 0;
    int count = 0;
    for (float x = x_min; x < x_max; x += 0.01) {
        for (float y = y_min; y < y_max; y += 0.01) {
            float z = plane_z_interp(x, y, plane_dist, plane_normal);
            zPlaneSum += z;
            count++;
            input_cloud_ptr->points.push_back(pcl::PointXYZ(x, y, z));
        }
    }
    //pcl::toROSMsg(*g_interpolated_plane_fit_cloud, g_interpolated_plane_fit_cloud_ros);
    //g_interpolated_plane_fit_cloud_ros.header.frame_id = "RBIP_frame";
    int n_fitted = input_cloud_ptr->points.size();
    input_cloud_ptr->width = n_fitted;
    input_cloud_ptr->height = 1;

    return zPlaneSum / count;
}

void write_table_transform_launchfile(Eigen::Affine3f affine_plane_wrt_cam,string fname) {
    ofstream outfile(fname.c_str());
    /*
     <launch>
       <node pkg="tf2_ros" type="static_transform_publisher" name="system_ref_frame" args="0 0 0 0 0 0 1 rear_wheel_frame system_ref_frame" />
     </launch>
     <node pkg="tf2_ros" type="static_transform_publisher" name="system_ref_frame" args="0 0 0 0 0 0 1 rear_wheel_frame system_ref_frame" />
     */
    string offset,quat_string;
    Eigen::Quaternionf orientation(affine_plane_wrt_cam.linear());
    quat_string=to_string(orientation.x())+" "+to_string(orientation.y())+" "+to_string(orientation.z())+" "+to_string(orientation.w());
    Eigen::Vector3f offset_vec = affine_plane_wrt_cam.translation();
    offset=to_string(offset_vec[0])+" "+to_string(offset_vec[1])+" "+to_string(offset_vec[2]);
    outfile<<"<launch>"<<endl;
    outfile<<"<node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"table_frame\" args=\""<<offset<<" "<<quat_string<<" camera_depth_optical_frame table_frame\" /> "<<endl;
    outfile<<"</launch>"<<endl;
    ROS_INFO_STREAM("calibrated launch file written to "<<fname<<endl);
}

//find points above table functions



//callback
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped amclPose){
	ROS_WARN("ENTERED AMCL CALLBACK");
	amcl_pose_data = amclPose;
	current.pose = amclPose.pose.pose;
	current.header = amclPose.header;
}



//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "append_path_client");
  ros::NodeHandle n;
  OdomTf odomTf(&n);
  ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
  ros::ServiceClient backup_client = n.serviceClient<std_srvs::Trigger>("new_backup");
  amcl_pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,amclCallback);
  geometry_msgs::Quaternion quat;
  std_srvs::Trigger trigger;
  
  ros::Subscriber pointcloud_subscriber = n.subscribe("/camera/depth_registered/points", 1, kinectCB);
  
  PclUtils pclUtils(&n); //instantiate a PclUtils object--a local library w/ some handy fncs
    
    //instantiate pointers to various clouds
    //pclKinect_clr_ptr will contain data from the from the image read in,
    //others are results of processing this original cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_plane_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    //will need ROS-style messages corresponding to the pointclouds
    sensor_msgs::PointCloud2 downsampled_cloud, ros_box_filtered_cloud; //here are ROS-compatible messages
    sensor_msgs::PointCloud2 ros_cloud_wrt_table,ros_cloud,ros_planar_cloud;
    
    vector<int> indices; //use this to identify points of interest, using indices of these points

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    fname = "kinect_snapshot.pcd";
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, planar_pts, downsampled_pcd, box_filted_pcd and table_frame_pts");

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = n.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = n.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = n.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt = n.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    ros::Publisher pubTableFrame = n.advertise<sensor_msgs::PointCloud2> ("table_frame_pts", 1);
  
  while (!client.exists()) {
    ROS_INFO("waiting for service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  mobot_pub_des_state::path path_srv;
  
  //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here

  // we want to add to points. one intermediate to have the robot move left towards docking station and one just before the docking station. 
      // it should drive straight from the previous point of interest.
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  geometry_msgs::Pose pose;
  pose.position.x = 0.0; // Center of robot aligned with center of dock
  pose.position.y = 0.412;
  pose.position.z = 0.0; // let's hope so!
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.x = 3.903 - distRobotFrontToCenter; // front of robot on center of dock
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);
      
  client.call(path_srv);

  geometry_msgs::PoseStamped goal = path_srv.request.path.poses[1];
  //geometry_msgs::PoseStamped current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
/*  
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH FIRST GOAL");
    ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
  
  ros::Duration(.5).sleep();
  //spin until obtain a snapshot
    ROS_INFO("waiting for kinect data");
    while (!got_kinect_image) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got snapshot; saving to file kinect_snapshot.pcd");
//    try{
	    pcl::io::savePCDFile("kinect_snapshot.pcd", *pclKinect_clr_ptr, true);//throws error why?
//    }catch(std::exception& e){
//    	ROS_WARN(e.what());
//    }

// need to create a service? to find block centroid
// Prepose arms
// Use block centroid and orientation to call the block_grabber action server
// We should be able to copy most of the code from block_grabber_action_client
// That's basically it, right?

/*   //This probly unnecessary since we don't really need to save the pcd to the disk
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, planar_pts, downsampled_pcd, box_filted_pcd and table_frame_pts");
    */
    
  ros::Duration(5).sleep();
  ros::spinOnce();

  ROS_WARN("About to back it up");
  backup_client.call(trigger);
  ROS_WARN("Backed up %s", trigger.response.success ? "successfully" : "unsuccessfully :(");
  goal = path_srv.request.path.poses[1];
  double phi = convertPlanarQuat2Phi(current.pose.orientation);
  goal.pose.position.x = current.pose.position.x - cos(phi);
  goal.pose.position.y = current.pose.position.y - sin(phi);
  goal.pose.orientation = current.pose.orientation;
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
/*
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO BACK UP AFTER FIRST GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);

    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
  ros::Duration(5).sleep();
  ros::spinOnce();
  
  pose.position.x = 0.542; // Center of robot aligned with center of dock
  pose.position.y = 0.0;
  pose.position.z = 0.0; // let's hope so!
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.y = 2.572 - distRobotFrontToCenter; // front of robot on center of dock
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);
      
  client.call(path_srv);

  goal = path_srv.request.path.poses[3];
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
/*
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH SECOND GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
  ros::Duration(5).sleep();
  ros::spinOnce();

  backup_client.call(trigger);
  goal = path_srv.request.path.poses[3];
  phi = convertPlanarQuat2Phi(current.pose.orientation);
  goal.pose.position.x = current.pose.position.x - cos(phi);
  goal.pose.position.y = current.pose.position.y - sin(phi);
  goal.pose.orientation = current.pose.orientation;
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
/*
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO BACK UP AFTER SECOND GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
  ros::Duration(5).sleep();
  ros::spinOnce();

  pose.position.x = 0.0; // Center of robot aligned with center of dock
  pose.position.y = 0.0;
  pose.position.z = 0.0; // let's hope so!
  quat = convertPlanarPhi2Quaternion(0);
  pose.orientation = quat;
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);

  pose.position.x = 0.0; // front of robot on center of dock
  pose_stamped.pose = pose;
  path_srv.request.path.poses.push_back(pose_stamped);
      
  client.call(path_srv);

  goal = path_srv.request.path.poses[5];
  //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
/*
  while(!poseToleranceCheck(current.pose,goal.pose)){
  	ROS_WARN("WAITING TO REACH END GOAL");
  	ROS_INFO("current.pose x,y %f,%f",current.pose.position.x, current.pose.position.y);
    ROS_INFO("goal.pose x,y %f,%f", goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("current.pose phi:%f",convertPlanarQuat2Phi(current.pose.orientation));
    ROS_INFO("goal.pose phi:%f",convertPlanarQuat2Phi(goal.pose.orientation));
    //current = xform_utils.get_pose_from_stamped_tf(odomTf.stfEstBaseWrtMap_);
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
  return 0;
}
