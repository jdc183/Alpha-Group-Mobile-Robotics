#include <find_block_service.h>


//bool got_kinect_image = false; //snapshot indicator
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

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
	ROS_INFO("got new selected kinect image");
	pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr);
	ROS_INFO("image has  %d * %d points", pclKinect_clr_ptr->width, pclKinect_clr_ptr->height);
}

// Main service callback
bool centroidCB(alpha_final::FindCentroidServiceRequest& request, alpha_final::FindCentroidServiceResponse& response){
	
	return true;
} 

//Main function 
int main(int argc, char **argv) {
	//ros init
	ros::init(argc, argv, "centroid_service");
	ros::NodeHandle n;
	
	PclUtils pclUtils(&n); //instantiate a PclUtils object--a local library w/ some handy fncs

	ros::ServiceServer service = n.advertiseService("centroid_service",centroidCB);

	ros::spin();

	return 0;
}
