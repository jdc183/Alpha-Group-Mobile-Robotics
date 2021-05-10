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
#include <alpha_final/FindCentroidService.h>
#include <alpha_final/ObjectGrabberService.h>

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


ros::ServiceClient find_centroid_client;
ros::ServiceClient grabber_client;
ros::ServiceClient dropper_client;



//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "append_path_client");
  ros::NodeHandle n;
  
  
  find_centroid_client = n.serviceClient<alpha_final::FindCentroidService> ("centroid_service",1);
  
  while (!find_centroid_client.exists()) {
    ROS_INFO("waiting for centroid service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  
  grabber_client = n.serviceClient<alpha_final::ObjectGrabberService> ("grabbbation_service",1);
  
  while (!grabber_client.exists()) {
    ROS_INFO("waiting for grabber service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  
  dropper_client = n.serviceClient<alpha_final::ObjectGrabberService> ("dropppation_service",1);
  
  while (!dropper_client.exists()) {
    ROS_INFO("waiting for dropper service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  
  alpha_final::ObjectGrabberService grabber_srv;
  alpha_final::FindCentroidService centroid_srv;
  
  
  ROS_INFO("Calling centroid service");
  
  find_centroid_client.call(centroid_srv);
  
  grabber_srv.request.des_pose = centroid_srv.response.output;
  
  ROS_INFO("Calling grabber service");
  
  grabber_client.call(grabber_srv);
  
  ROS_INFO("Done grabbin' start droppin'");
  
  dropper_client.call(grabber_srv);
  
  ROS_INFO("Done droppin'");

  return 0;
}
