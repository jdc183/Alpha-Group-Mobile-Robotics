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


//Global variables
int g_ans;
using namespace std;
PclUtils *g_pcl_utils_ptr;

//Helper functions
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

            tfListener.lookupTransform("torso", "table_frame", ros::Time(0), table_frame_wrt_bot_stf);
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

            tfListener.lookupTransform("head", "camera_depth_optical_frame", ros::Time(0), cam_to_bot);
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
    ROS_INFO("tf is good for camera w/rt robot");
    xformUtils.printStampedTf(cam_to_bot);
    

    tf::Transform cam_to_bot2 = xformUtils.get_tf_from_stamped_tf(cam_to_bot);
    
    affine_cam_to_bot = xformUtils.transformTFToAffine3f(cam_to_bot2);

    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return cam_to_bot2;
}

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

void findAboveTable(){
	ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_robot_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_ptr2(new pcl::PointCloud<pcl::PointXYZRGB>);


    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, table_frame_pts and pts_above_table");

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubTableFrame = nh.advertise<sensor_msgs::PointCloud2> ("table_frame_pts", 1);
    ros::Publisher pubPointsAboveTable = nh.advertise<sensor_msgs::PointCloud2> ("pts_above_table", 1);

    sensor_msgs::PointCloud2 ros_cloud_wrt_table,ros_pts_above_table,ros_cloud_orig, ros_cloud_wrt_table2, ros_pts_above_table2;
    //ros_cloud_wrt_table.header.frame_id = "table_frame";

    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud_orig); //convert from PCL cloud to ROS message this way
    ros_cloud_orig.header.frame_id = "camera_depth_optical_frame";


    //find the transform of table w/rt camera and convert to an affine
    Eigen::Affine3f affine_table_wrt_cam, affine_cam_wrt_table;
    affine_table_wrt_cam = get_table_frame_wrt_camera();
    affine_cam_wrt_table=affine_table_wrt_cam.inverse();

    /*
    //find the transform of table w/rt robot torso and convert to an affine
    Eigen::Affine3f affine_table_wrt_robot, affine_robot_wrt_table;
    affine_table_wrt_robot = get_table_frame_wrt_robot();
    affine_robot_wrt_table = affine_table_wrt_robot.inverse();*/

    pclUtils.transform_cloud(affine_cam_wrt_table, pclKinect_clr_ptr, output_cloud_wrt_table_ptr);
    pcl::toROSMsg(*output_cloud_wrt_table_ptr, ros_cloud_wrt_table);
    ros_cloud_wrt_table.header.frame_id = "table_frame";

    /*
    pclUtils.transform_cloud(affine_robot_wrt_table, pclKinect_clr_ptr, output_cloud_wrt_robot_ptr);
    pcl::toROSMsg(*output_cloud_wrt_robot_ptr, ros_cloud_wrt_table2);
    ros_cloud_wrt_table2.header.frame_id = "table_frame";*/    
    
    //cout<<"enter 1: ";
    //cin>>g_ans;
    //find indicies of points above table:
    vector<int> indices;
    find_indices_of_plane_from_patch(output_cloud_wrt_table_ptr, indices);
    pcl::copyPointCloud(*output_cloud_wrt_table_ptr, indices, *pts_above_table_ptr); //extract these pts into new cloud

    /*
    vector<int> indices2;
    find_indices_of_plane_from_patch(output_cloud_wrt_robot_ptr, indices2);
    pcl::copyPointCloud(*output_cloud_wrt_robot_ptr, indices2, *pts_above_table_ptr2); //extract these pts into new cloud
    */
// PS9 additions
    
    Eigen::Vector3f c1;
    c1 = pclUtils.compute_centroid(*pts_above_table_ptr);
    
    tf::Transform cam_to_bot = cam_to_robot();

    double botx = c1.x() + cam_to_bot.getOrigin().x();
    double boty = c1.y() + cam_to_bot.getOrigin().y();
    double botz = c1.z() + cam_to_bot.getOrigin().z();

    /*
    Eigen::Vector3f c2;
    c2 = pclUtils.compute_centroid(*pts_above_table_ptr2);
    */

    //Transform c1 into torso frame
    
    ROS_INFO("block centroid xyz: %f,%f,%f",c1.x(),c1.y(), c1.z());
    //ROS_INFO("block centroid xyz torso: %f,%f,%f",c2.x(),c2.y(), c2.z());
    ROS_INFO("block centroid xyz in head frame: %f,%f,%f",botx,boty,botz);
// end PS9 additions
    
    pcl::toROSMsg(*pts_above_table_ptr, ros_pts_above_table);
    //pcl::toROSMsg(*pts_above_table_ptr2, ros_pts_above_table2);
    ros_pts_above_table.header.frame_id = "table_frame";     
    //ros_pts_above_table2.header.frame_id = "table_frame";     

/*
    while (ros::ok()) {

        pubTableFrame.publish(ros_cloud_wrt_table);
        //pubTableFrame.publish(ros_cloud_wrt_table2);
        pubCloud.publish(ros_cloud_orig); // will not need to keep republishing if display setting is persistent
        pubPointsAboveTable.publish(ros_pts_above_table);
        //pubPointsAboveTable.publish(ros_pts_above_table2);
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.3).sleep();
    }

    return 0;*/
}

void findTableFrame(){

	PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    
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
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, planar_pts, downsampled_pcd, box_filted_pcd and table_frame_pts");

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    ros::Publisher pubTableFrame = nh.advertise<sensor_msgs::PointCloud2> ("table_frame_pts", 1);

    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << -0.4, -0.5, 0.2;
    box_pt_max << 0.4, 0.0, 2;


    int nsamps = downsampled_kinect_ptr->points.size();
    ROS_INFO("num pts in downsampled cloud: %d", nsamps);
    pclUtils.box_filter(downsampled_kinect_ptr, box_pt_min, box_pt_max, indices);

    pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud); //convert to ros message for publication and display

    //find plane fit to box-filtered cloud
    Eigen::Vector3f plane_normal;
    double plane_dist;    
    pclUtils.fit_points_to_plane(box_filtered_cloud_ptr, plane_normal, plane_dist);
    ROS_INFO_STREAM("plane_normal = " << plane_normal.transpose() << "; plane_dist = " << plane_dist << endl);
    createInterpolatedPlaneCloud(fitted_plane_ptr, plane_normal, plane_dist, box_pt_max(0), box_pt_max(1), box_pt_min(0), box_pt_min(1));
    int n_fitted = fitted_plane_ptr->points.size();

    ROS_INFO("fitted plane has %d points", n_fitted);
    pcl::toROSMsg(*fitted_plane_ptr, ros_planar_cloud);
    ros_planar_cloud.header.frame_id = "camera_depth_optical_frame";

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
    cout << "O_plane_wrt_cam: " << O_plane_wrt_cam.transpose() << endl;
    cout << "R_plane_wrt_cam: " << endl << R_plane_wrt_cam << endl;
    affine_cam_wrt_plane = affine_plane_wrt_cam.inverse();

    //convert all points in box_filtered_cloud_ptr into the new table frame, with output in output_cloud_wrt_table_ptr
    pclUtils.transform_cloud(affine_cam_wrt_plane, box_filtered_cloud_ptr, output_cloud_wrt_table_ptr);
    
    //prepare to display the result of the transformed data
    pcl::toROSMsg(*output_cloud_wrt_table_ptr, ros_cloud_wrt_table);
    //pointcloud does not tell us the ROS frame; must install manually
    ros_cloud_wrt_table.header.frame_id = "table_frame";    
    
    //cout<<"enter 1: "; //poor-man's breakpoint
    //cin>>g_ans;
    ROS_INFO("analyzing transformed cloud: ");
    //test: fit a plane to the surviving points
    //we SHOULD find that the surface normal is [0;0;1] and the offset distance is zero
    pclUtils.fit_points_to_plane(output_cloud_wrt_table_ptr, plane_normal, plane_dist);

    //this is somewhat unusual: write out a ROS launchfile that contains the results herein,
    // describing the transform between camera frame and table frame
    //subsequently,  run this launch file; should be valid at station 1, as long as the camera pose does not change w/rt torso
    // and table height and orientation remain the same relative to the torso
    //note that this launchfile will be written to the current directory.
    // will need to move it into some ROS package to launch it
    string launchfile_name = "table_frame_wrt_cam.launch";
    write_table_transform_launchfile(affine_plane_wrt_cam,launchfile_name);
/*
    //publish all of the computed clouds, so they can be visualized in rviz
    while (ros::ok()) {

        pubTableFrame.publish(ros_cloud_wrt_table);
        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        pubBoxFilt.publish(ros_box_filtered_cloud);
        ros::spinOnce(); //not really needed here, but required if receiving pointcloud messages
        ros::Duration(0.3).sleep();
    }

    return 0;*/

}

//Callback functions
geometry_msgs::Pose centroidCB(alpha_final::FindCentroidServiceRequest& request, alpha_final::FindCentroidServiceResponse& response){

} 

//Main function 
int main(int argc, char **argv) {
	//ros init
	ros::init(argc, argv, "centroid_service");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("centroid_service",centroidCB);

	ros::spin();

	return 0;
}
