
// Node designed to inspect a box on the left side of the robot
// This is to detect that a wall is present nearby on the left of the robot

// We want to subscribe to the lidar and only look at what is on the left side of the robot. 
// Assuming the unit circle follows the robot conventions with 0 deg facing forward, our range of interest is between 5pi/6 and 7pi/6
// We do not want to set off an alarm, but we do want to create a 'dummy' variable that triggers when the robot will need to rotate because there is no wall on the LHS
// This variable will be reset in node 4 instead of this code (i think)

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h>

#define PI M_PI

const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
//int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool laser_initialized = false;

//assuming ccw indexing
double angle_min_alarm = PI/3;//Rightmost angle of interest
double angle_max_alarm = 2*PI/3;//Leftmost angle of interest
double range_min_alarm = 0.06;//minimum distance of interest
double range_max_alarm = 2.0;//maximum distance of interest
// triggers when distance is greater than 2 meters
int max_ping_index = 0;//Index of rightmost angle
int min_ping_index = 0;//Index of leftmost angle

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (!laser_initialized)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
//        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        
        //Set ping indices to correspond to angles of interest
        max_ping_index = (int) ((angle_max_alarm - angle_min_)/angle_increment_);
        min_ping_index = (int) ((angle_min_alarm - angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup");
        
        laser_initialized = true;
    }
    
    laser_alarm_=false;

    //scans lidar data within ping indices to find ranges
    for (int ping_index_ = min_ping_index; ping_index_ <= max_ping_index; ping_index_++){
      ping_dist_in_front_ = laser_scan.ranges[ping_index_];
      ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
    
      //checks if each lidar data point is within alarm distance
      if (ping_dist_in_front_<range_max_alarm && ping_dist_in_front_ > range_min_alarm) {
          laser_alarm_=true;
      } 
    }

    //sends warning if data point trips lidar alarm
    if (laser_alarm_){
        ROS_WARN("MY SAFETY NET IS GONE WHERE AM I!");
        laser_alarm_ = false;
    }

   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigator"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("left_lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("left_lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}