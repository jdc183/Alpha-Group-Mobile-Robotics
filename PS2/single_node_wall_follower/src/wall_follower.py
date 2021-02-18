#!/usr/bin/env python

# roslaunch xml file: package, type = executable.py, name = node, elements
import roslib; roslib.load_manifest("single_node_wall_follower")
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Callback for receipt of laser data
def on_lidar(laser):
	global pub

	cmd_vel = Twist()# Create a new Twist instance
	cmd_vel.angular.z = 0.0	# Set rotational speed to zero (for now)
	cmd_vel.linear.x = 0.5 # Set the linear speed to a reasonable constant

	# Modify linear and rotational velocities according to lidar
	for n in range(len(laser.ranges)):
	 	angle = laser.angle_increment*n + laser.angle_min # calculate angle of each laser beam
	 	dist = laser.ranges[n]
	 	if dist > 1.8:
	 		dist = 1.8
	 	#Compute a cartesian point cloud from the lidar data
	 	x = dist*math.cos(angle)
	 	y = dist*math.sin(angle)

	 	#Velocity adjustments are calculated by mapping each point 
	 	linadjust = -1/((((8*(x-.25))**20)+1)*((4*y)**20+1))
	 	angadjust = -1/((((4*(x-.5))**20)+1)*((4*y)**20+1))+1/((((4*(x-.5))**20)+1))*(1/(((1*(y-1.5))**20+1)))+1/((((2*(x-.75))**20)+1))*(-1/(((16*(y-0.25))**20)+1))

	 	cmd_vel.linear.x += 10*linadjust/len(laser.ranges)
	 	cmd_vel.angular.z += 20*angadjust/len(laser.ranges)

	# Don't let either velocity exceed reasonable bounds or the sim will stop working
	if cmd_vel.linear.x > 1:
		cmd_vel.linear.x = 1
	elif cmd_vel.linear.x < -1:
		cmd_vel.linear.x = -1
	if cmd_vel.angular.z > 6:
		cmd_vel.angular.z = 6
	elif cmd_vel.angular.z < -6:
		cmd_vel.angular.z = -6


        
	pub.publish(cmd_vel)

# Main
def wall_follow():
	global pub
	robot = "robot0"
	steering = "des_vel"

	# Set up twist publisher and laser subscriber
	pub = rospy.Publisher(str(robot) + "/cmd_vel", Twist, queue_size=10)
	rospy.init_node("single_node_wall_follower", anonymous=True)
	lidar_sub = rospy.Subscriber(str(robot) + "/laser_0", LaserScan, on_lidar)
	rospy.spin()

def mapXYtoLinearSpeed(x,y):
	return -1/((((8*(x-.25))**20)+1)*((4*y)**20+1))

# Here's a visual representation of what this map looks like
# X is where the robot is, - is roughly -1, + roughly +1, and space is 0
#  ^ y                           
#  |                             
#  |                             
#  |       x                     
#  L------->                     
#                                
#              XXXX ---          
#             XXXXXX---          
#             XXXXXX---          
#              XXXX ---          
                           


def mapXYtoAngularVelocity(x,y):
	return -1/((((4*(x-.5))**20)+1)*((4*y)**20+1))+1/((((4*(x-.5))**20)+1))*(1/(((1*(y-1.5))**20+1)))+1/((((2*(x-.75))**20)+1))*(-1/(((16*(y-0.25))**20)+1))

#Here's a visual representation using the same method as above
#                   ++++++       
#                   ++++++       
#                   ++++++       
#                   ++++++       
#                   ++++++        
#                   ++++++       
#                   ++++++       
#                   ++++++       
#                   ++++++       
#                   ++++++        
#                   --------------
#              XXXX ------        
#             XXXXXX------        
#             XXXXXX------        
#              XXXX ------        

if __name__ == "__main__":
	try:
		wall_follow()
	except rospy.ROSInterruptException:
		pass
