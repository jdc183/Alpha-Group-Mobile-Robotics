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

#include <alpha_final/FindCentroidService.h>

//Global variables


//Helper functions


//Callback functions
geometry_msgs::Pose centroidCB(alpha_final::FindCentroidServiceRequest& request, alpha_final::FindCentroidServiceResponse& response){
	
} 

//Main function 
int main(int argc, char **argv) {
	//ros init
	ros::init(argc, argv, "centroid_service");
	ros::NodeHandle n;

	ros::ServiceClient pubdesClient = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
	ros::ServiceServer service = n.advertiseService("centroid_service",centroidCB);

	ros::spin();

	return 0;
}
