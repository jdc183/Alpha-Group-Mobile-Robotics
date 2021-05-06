//include statements
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <queue>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>
#include <pcl_utils/pcl_utils.h>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <mobot_pub_des_state/path.h>

#include <alpha_final/NavService.h>
#include <alpha_final/BackupService.h>
#include <alpha_final/FindCentroidService.h>

//pcl includes:
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/PCLHeader.h>



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


bool got_kinect_image = false; //snapshot indicator

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //downsampled version of above
pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //box_filtered version of above (should hopefully just include table)
pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_plane_ptr(new pcl::PointCloud<pcl::PointXYZ>); //Planar point cloud fitted to above

pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

sensor_msgs::PointCloud2 intermediate_cloud_ros;
ros::Publisher intermediateCloudPublisher;  // publishes intermediate clouds during the process of finding the block

//Helper functions

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

//Callback functions

//Get point cloud from kinect
void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    ROS_INFO("kinectCB!!!");
    if (!got_kinect_image) { // once only, to keep the data stable
	ROS_INFO("got new selected kinect image");
	pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
	ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
	got_kinect_image = true;
    }
}

// Main service callback
bool centroidCB(alpha_final::FindCentroidServiceRequest& request, alpha_final::FindCentroidServiceResponse& response){

	ros::NodeHandle n2;
	PclUtils pclUtils(&n2);	
	
	ROS_INFO(" ");
	ROS_INFO("centroidCB!!!");
	ROS_INFO(" ");
	
	
	got_kinect_image = false;
	ROS_INFO("waiting for kinect data");
	while (!got_kinect_image) {
	    ROS_INFO("waiting...");
	    ros::spinOnce();
	    ros::Duration(0.5).sleep();
	}
	pcl::io::savePCDFile("kinect_snapshot1.pcd", *pclKinect_clr_ptr, true);
	
	/*******************************************************************************************************
	Find the table frame
	*******************************************************************************************************/
	ROS_INFO("Find the table frame");
	
	vector<int> indices; //use this to identify points of interest, using indices of these points
	
	ROS_INFO("start voxel filtering");
	pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    	vox.setInputCloud(pclKinect_clr_ptr);

    	vox.setLeafSize(0.02f, 0.02f, 0.02f);
    	vox.filter(*downsampled_kinect_ptr);
	ROS_INFO("done voxel filtering");
	
	Eigen::Vector3f box_pt_min, box_pt_max;
    	box_pt_min << -0.4, -0.5, 0.2;
    	box_pt_max << 0.4, 0.0, 2;
	
	int nsamps = downsampled_kinect_ptr->points.size();
    	ROS_INFO("num pts in downsampled cloud: %d", nsamps);
	
	pclUtils.box_filter(downsampled_kinect_ptr, box_pt_min, box_pt_max, indices);

    	pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud
    	//the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
	
	//find plane fit to box-filtered cloud
    	Eigen::Vector3f plane_normal;
    	double plane_dist;    
	pclUtils.fit_points_to_plane(box_filtered_cloud_ptr, plane_normal, plane_dist);
	ROS_INFO_STREAM("plane_normal = " << plane_normal.transpose() << "; plane_dist = " << plane_dist << endl);
	createInterpolatedPlaneCloud(fitted_plane_ptr, plane_normal, plane_dist, box_pt_max(0), box_pt_max(1), box_pt_min(0), box_pt_min(1));
	int n_fitted = fitted_plane_ptr->points.size();
	
    	ROS_INFO("fitted plane has %d points", n_fitted);
    	fitted_plane_ptr->header.frame_id = "camera_depth_optical_frame";
	
	//create a frame on the identified plane:
	Eigen::Vector3f z_plane_wrt_cam, y_plane_wrt_cam, x_plane_wrt_cam;
	Eigen::Matrix3f R_plane_wrt_cam;
	Eigen::Vector3f O_plane_wrt_cam;
	Eigen::Vector3f z_cam_wrt_cam,x_cam_wrt_cam;
	z_cam_wrt_cam << 0, 0, 1;
	x_cam_wrt_cam <<1,0,0;
	O_plane_wrt_cam = plane_normal*plane_dist;
	z_plane_wrt_cam = plane_normal;
	
	//use x_plane = -y_camera projected onto identified plane:
	//equiv, construct x_plane = -x_cam x plane_normal
	x_plane_wrt_cam = -x_cam_wrt_cam.cross(plane_normal);
	x_plane_wrt_cam = x_plane_wrt_cam/x_plane_wrt_cam.norm(); //axes must be unit length
	y_plane_wrt_cam = z_plane_wrt_cam.cross(x_plane_wrt_cam);
	
	
	R_plane_wrt_cam.col(0) = x_plane_wrt_cam;
	R_plane_wrt_cam.col(1) = y_plane_wrt_cam;
	R_plane_wrt_cam.col(2) = z_plane_wrt_cam;
	
	//put the above into an affine transform:
	Eigen::Affine3f affine_plane_wrt_cam, affine_cam_wrt_plane;
	affine_plane_wrt_cam.linear() = R_plane_wrt_cam;
	affine_plane_wrt_cam.translation() = O_plane_wrt_cam;
	
	affine_cam_wrt_plane = affine_plane_wrt_cam.inverse();

	//convert all points in box_filtered_cloud_ptr into the new table frame, with output in output_cloud_wrt_table_ptr
	pclUtils.transform_cloud(affine_cam_wrt_plane, box_filtered_cloud_ptr, output_cloud_wrt_table_ptr);
	
	string launchfile_name = "table_frame_wrt_cam.launch";
	write_table_transform_launchfile(affine_plane_wrt_cam,launchfile_name);

	
	pcl::toROSMsg(*output_cloud_wrt_table_ptr, intermediate_cloud_ros);
	
	while(ros::ok()){
		intermediateCloudPublisher.publish(intermediate_cloud_ros);
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	
	return true;
}

//Main function 
int main(int argc, char **argv) {
	//ros init
	ros::init(argc, argv, "centroid_service");
	ros::NodeHandle n;
	
//	PclUtils pclUtils(&n); //instantiate a PclUtils object--a local library w/ some handy fncs

	ros::ServiceServer service = n.advertiseService("centroid_service",centroidCB);
	
	ros::Subscriber pointcloud_subscriber = n.subscribe("/camera/depth_registered/points", 1, kinectCB);
	
	intermediateCloudPublisher = n.advertise<sensor_msgs::PointCloud2> ("intermediate_points", 1);
	
	ros::spin();

	return 0;
}
