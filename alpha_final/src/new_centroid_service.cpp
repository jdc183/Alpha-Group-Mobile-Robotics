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

#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */


bool got_kinect_image = false; //snapshot indicator
//************
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_robot_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_wrt_torso(new pcl::PointCloud<pcl::PointXYZRGB>);
//***************
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //downsampled version of above
pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //box_filtered version of above (should hopefully just include table)
pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_plane_ptr(new pcl::PointCloud<pcl::PointXYZ>); //Planar point cloud fitted to above

pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

sensor_msgs::PointCloud2 intermediate_cloud_ros;
sensor_msgs::PointCloud2 ros_pts_above_table;
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

// *****************
void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, vector<int> &indices) {

    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    // changed from 0.1 to 0.2; will use this as a filter until only the block is left and other table parts are filtered out.
    pass.setFilterLimits(0.025, 0.05); //retain points with z values between these limits
    pass.filter(indices); //  this will return the indices of the points in given cloud that pass our test
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}

Eigen::Affine3f get_table_frame_wrt_camera() {
    bool tferr = true;
    int ntries = 0;
    XformUtils xformUtils;
    tf::TransformListener tfListener;
    tf::StampedTransform table_frame_wrt_cam_stf;

    Eigen::Affine3f affine_table_wrt_camera;
    while (tferr) {
        tferr = false;
        try {

            tfListener.lookupTransform("camera_depth_optical_frame", "table_frame", ros::Time(0), table_frame_wrt_cam_stf);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5) {
                 ROS_WARN("did you launch robot's table_frame_wrt_cam.launch?");
                 ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for table w/rt camera");
    xformUtils.printStampedTf(table_frame_wrt_cam_stf);
    

    tf::Transform table_frame_wrt_cam_tf = xformUtils.get_tf_from_stamped_tf(table_frame_wrt_cam_stf);
    
    affine_table_wrt_camera = xformUtils.transformTFToAffine3f(table_frame_wrt_cam_tf);

    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return affine_table_wrt_camera;
}

Eigen::Affine3f get_table_frame_wrt_robot() {
    bool tferr = true;
    int ntries = 0;
    XformUtils xformUtils;
    tf::TransformListener tfListener;
    tf::StampedTransform table_frame_wrt_bot_stf;

    Eigen::Affine3f affine_table_wrt_robot;
    while (tferr) {
        tferr = false;
        try {

            tfListener.lookupTransform("system_ref_frame", "table_frame", ros::Time(0), table_frame_wrt_bot_stf);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5) {
                 ROS_WARN("did you launch robot's transform.launch?");
                 ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for table w/rt robot");
    xformUtils.printStampedTf(table_frame_wrt_bot_stf);
    

    tf::Transform table_frame_wrt_bot_tf = xformUtils.get_tf_from_stamped_tf(table_frame_wrt_bot_stf);
    
    affine_table_wrt_robot = xformUtils.transformTFToAffine3f(table_frame_wrt_bot_tf);

    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return affine_table_wrt_robot;
}

tf::Transform cam_to_robot(){
    bool tferr = true;
    int ntries = 0;
    XformUtils xformUtils;
    tf::TransformListener tfListener;
    tf::StampedTransform cam_to_bot;

    Eigen::Affine3f affine_cam_to_bot;
    while (tferr) {
        tferr = false;
        try {

            tfListener.lookupTransform("torso", "camera_depth_optical_frame", ros::Time(0), cam_to_bot);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5) {
                 ROS_WARN("did you launch robot's transform.launch?");
                 ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for camera w/rt robot");
    xformUtils.printStampedTf(cam_to_bot);
    

    tf::Transform cam_to_bot2 = xformUtils.get_tf_from_stamped_tf(cam_to_bot);
    
    affine_cam_to_bot = xformUtils.transformTFToAffine3f(cam_to_bot2);

    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return cam_to_bot2;
}


//******************
//Callback functions

//Get point cloud from kinect
void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    //ROS_INFO("kinectCB!!!");
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
	
	//ROS_INFO(" ");
	ROS_INFO("centroidCB!!!");
	//ROS_INFO(" ");
	
	
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
	output_cloud_wrt_table_ptr->header.frame_id = "table_frame";
	
	int i = system("alphaws");
	i = system("roscd alpha_final");
	
	string launchfile_name = "table_frame_wrt_cam.launch";
	write_table_transform_launchfile(affine_plane_wrt_cam,launchfile_name);
	
	
	i = system("rosnode kill /table_frame");
	i = system("roslaunch table_frame_wrt_cam.launch &");
	i = system("alphaws");

	
	//thursday lab additions
	find_indices_of_plane_from_patch(output_cloud_wrt_table_ptr, indices);
	pcl::copyPointCloud(*output_cloud_wrt_table_ptr, indices, *pts_above_table_ptr); //extract these pts into new cloud
	
	Eigen::Vector3f c1;
	
    	c1 = pclUtils.compute_centroid(*pts_above_table_ptr);
    
    	tf::Transform cam_to_bot = cam_to_robot();
    	
//    	/*
    	Eigen::Affine3f affine_table_wrt_torso = get_table_frame_wrt_robot();
    	pclUtils.transform_cloud(affine_table_wrt_torso, pts_above_table_ptr, pts_above_table_wrt_torso);
	Eigen::Vector3f c2 = pclUtils.compute_centroid(*pts_above_table_wrt_torso);
//	*/

    	double botx = c1.x() + cam_to_bot.getOrigin().x();
    	double boty = c1.y() + cam_to_bot.getOrigin().y();
    	double botz = c1.z() + cam_to_bot.getOrigin().z();
    	
    	//Transform c1 into torso frame
    
	ROS_INFO("block centroid xyz: %f,%f,%f",c1.x(),c1.y(), c1.z());
	ROS_INFO("block centroid xyz torso: %f,%f,%f",c2.x(),c2.y(), c2.z());
	ROS_INFO("block centroid xyz translated origin: %f,%f,%f",botx,boty,botz);
	// end PS9 additions
	    
	pcl::toROSMsg(*pts_above_table_ptr, ros_pts_above_table);
	//pcl::toROSMsg(*pts_above_table_ptr2, ros_pts_above_table2);
	ros_pts_above_table.header.frame_id = "table_frame";
	
	
	

	//end thursday lab additions
	
	
	//pcl::toROSMsg(*output_cloud_wrt_table_ptr, intermediate_cloud_ros);
	
	
	/*
	while(ros::ok()){
		intermediateCloudPublisher.publish(ros_pts_above_table);
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	*/
	response.output.position.x = c2.x();
	response.output.position.y = c2.y();
	response.output.position.z = c2.z();
	//0.947, 0.321, 0.014, 0.005
	response.output.orientation.x = 0;
	response.output.orientation.y = 0;
	response.output.orientation.z = 0.707;
	response.output.orientation.w = 0.707;
	
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
