// based off of make_and_display_ellipse.cpp
// (example of creating a point cloud and publishing it for rviz display)
// for guidance, see lecture 18 starting at 21:42 - 45:30. See make_clouds_for_blocks and 
// 46:28 - 1:23:31 discussion of find_plane_fixed_bounds.... not in learning_ros. 
        // fits points into a plane that we specify. 
        // helps find the plane normal and the plane offset

// starting with lecture 19
// 10:30 skip to around 40 minutes to see if this is doing the part of the lab i want to do

// skipping to lecture 20 because i did not see the info i needed in lecture 19
// 3:32 goes through the lab
// talks about part 1 until 12:24
// part 2 discussion lessgoooo; 12:24 - 
    // need frame associated with the tabletop 
    // want to find a transform that corresponds to the surface of the table top so that we can find the objects that are sitting on the table
    // add a cm to the elevation of the given recorded info to not hit the table when grabbing the blocks
    // can get location of robot's hand (cartesian representation) by running function: rosrun tf tf_echo frame1name frame2name
        // spews out pose relationship between the two specified frames
    // NG We need the dimensions of the green block to do this assignment?
    // transform tabletop to a normal to camera z frame in order to not miss any targets due to the table incline and the  camera thinking the object is submerged because iit iis lower than it is expecting
        // try it wiithout this extra transform initially?
    // process the point cloud images to find blocks
    // establish a coordinate transform such that you get the translation and rotation outputs based off of the image processing
        // only need the translation output for this assignment
    // height should always be -0.186 mm or around there (if torso axis is perpendicular to the floor)
    // how to do this: minute 25:00
        // rosrun table_transform find_points_above_table
            // block#.pcd
            // make sure transform.launch was ran to do this
                //roslaunch table_transform table_frame_wrt_cam.launch
                    // NG should this be the transform we wrote in part 1?
                // can switch to table frame in rviz
                    //origin lies in the plane of the table with relationship between the camera and the table frame
                // switch from pcd to points_above_table
                    // dominant cluster will be the block that we care about
                    // property of the block is that it is sitting on the table, so it will be higher than the table
            // tweak find points above table to filter out the stuff that is not the block (about 30 minutes in)
        // new package = table transform (31:55)
        


// 20 April 2021

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>


using namespace std;

//a function to populate a pointCloud and a colored pointCloud;
// provide pointers to these, and this function will fill them with data
void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr) {
    //example ellipse, borrowed from: http://docs.ros.org/hydro/api/pcl/html/pcl__visualizer__demo_8cpp_source.html
    // make an ellipse extruded along the z-axis. The color for
    // the XYZRGB cloud will gradually go from red to green to blue.
    
    uint8_t r(255), g(15), b(15); //declare and initialize red, green, blue component values
    
    //here are "point" objects that are compatible as building-blocks of point clouds
    pcl::PointXYZ basic_point; // simple points have x,y,z, but no color
    pcl::PointXYZRGB point; //colored point clouds also have RGB values

    for (float z = -1.0; z <= 1.0; z += 0.05) //build cloud in z direction
        // -1m to +1m in increments of 5cm
    {
        // color is encoded strangely, but efficiently.  Stored as a 4-byte "float", but
        // interpreted as individual byte values for 3 colors
        // bits 0-7 are blue value, bits 8-15 are green, bits 16-23 are red; 
        // Can build the rgb encoding with bit-level operations:
        uint32_t rgb = (static_cast<uint32_t> (r) << 16 |
                static_cast<uint32_t> (g) << 8 | static_cast<uint32_t> (b));
        
        // and encode these bits as a single-precision (4-byte) float:
        float rgb_float = *reinterpret_cast<float*> (&rgb);
        
        //using fixed color and fixed z, compute coords of an ellipse in x-y plane
        for (float ang = 0.0; ang <= 2.0 * M_PI; ang += 2.0 * M_PI / 72.0) {
            //choose minor axis length= 0.5, major axis length = 1.0
            // compute and fill in components of point
            basic_point.x = 0.5 * cosf(ang); //cosf is cosine, operates on and returns single-precision floats
            basic_point.y = sinf(ang);
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point); //append this point to the vector of points

            //use the same point coordinates for our colored pointcloud      
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            //but also add rgb information
            point.rgb = rgb_float; //*reinterpret_cast<float*> (&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0) //alter the color smoothly in the z direction
        {
            r -= 12; //less red
            g += 12; //more green
        } else {
            g -= 12; // for positive z, lower the green
            b += 12; // and increase the blue
        }
    }
    
    //these will be unordered point clouds, i.e. a random bucket of points
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1; //height=1 implies this is not an "ordered" point cloud
    basic_cloud_ptr->header.frame_id = "camera"; // need to assign a frame id

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->header.frame_id = "camera";        

}

int main(int argc, char** argv) {
    // block_coords is replacing ellipse. need to declare this topic somewhere NG
    ros::init(argc, argv, "block_coords"); //node name
    ros::NodeHandle nh;

    // create some point-cloud objects to hold data
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //no color. creates point
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //colored. creates point

    cout << "Generating example point-cloud ellipse.\n\n";
    cout << "view in rviz; choose: topic= ellipse; and fixed frame= camera" << endl;
    
    // -----use fnc to create example point clouds: basic and colored-----
    make_clouds_for_blocks(basic_cloud_ptr, point_cloud_clr_ptr);

    // we now have "interesting" point clouds in basic_cloud_ptr and point_cloud_clr_ptr
    //let's publish the colored point cloud in a ROS-compatible message; here's a publisher...
    // we'll publish to topic "ellipse"
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/block_coords", 1);
    sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible pointCloud message
    //we'll publish the colored point cloud; 
    pcl::toROSMsg(*point_cloud_clr_ptr, ros_cloud); //convert from PCL to ROS type this way

    //publish the ROS-type message; can view this in rviz on topic "/ellipse"
    //BUT need to set the Rviz fixed frame to "camera"
    while (ros::ok()) {
        pubCloud.publish(ros_cloud);
        ros::Duration(0.5).sleep(); //keep refreshing the publication periodically
    }
    return 0;
}
