<<<<<<< HEAD
#!/usr/bin/env python3
=======
#!/usr/bin/env python

# roslaunch xml file: package, type = executable.py, name = node, elements
>>>>>>> 261f66413d5a1f002c067bc1a05f1a2de0eec814
import roslib; roslib.load_manifest("single_node_wall_follower")
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global linVelGain
global angVelGain

# These variables determine how sensitive the throttle and steering
# are to the mapped lidar data
linVelGain = 10;
angVelGain = 20;

# Callback for receipt of laser data
def on_lidar(laser):
	global pub
	global linVelGain
	global angVelGain

	cmd_vel = Twist()# Create a new Twist instance
	cmd_vel.angular.z = 0.0	# Set rotational speed to zero (for now)
	cmd_vel.linear.x = 0.5 # Set the linear speed to a reasonable constant

	# Modify linear and rotational velocities according to lidar
	for n in range(len(laser.ranges)):
	 	angle = laser.angle_increment*n + laser.angle_min # calculate angle of each laser beam
	 	dist = laser.ranges[n] # read in the lenght of each laser beam

	 	# These two lines are mostly for dealing with Inf distances
	 	if dist > 1.8:
	 		dist = 1.8

	 	# Compute a cartesian point cloud from the lidar data
	 	x = dist*math.cos(angle)
	 	y = dist*math.sin(angle)

	 	# Velocity adjustments are calculated by mapping each point 
	 	linadjust = mapXYtoLinearSpeed(x,y)
	 	angadjust = mapXYtoAngularVelocity(x,y)

	 	# Set the velocities according to the maps
	 	cmd_vel.linear.x += linVelGain*linadjust/len(laser.ranges)
	 	cmd_vel.angular.z += angVelGain*angadjust/len(laser.ranges)

	# Don't let either velocity exceed reasonable bounds or the sim will stop working
	if cmd_vel.linear.x > 1:
		cmd_vel.linear.x = 1
	elif cmd_vel.linear.x < -1:
		cmd_vel.linear.x = -1
	if cmd_vel.angular.z > 6:
		cmd_vel.angular.z = 6
	elif cmd_vel.angular.z < -6:
		cmd_vel.angular.z = -6


    # command the robot
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



# It would be more efficient to use booleans to check whether the wall is in specific
# regions around the robot, but polynomials are continuous and allow for smoother feedback.
# Also, the fact that the robot's behavior is based on a weighted average of all the laser
# measurments means that it would me much less sensitive to measurement noise and glitches.

# A rational polynomial map to compute adjustments to the forward velocity based
# on the x,y coordinates of each obstruction around it
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
# Basically, the robot will always drive forward, but will slow down, stop, or even backtrack when something is
# directly in front of it

# A rational polynomial map to compute adjustments to the angular velocity based
# on the x,y coordinates of each obstruction around it
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
#                   -------------
#              XXXX ------        
#             XXXXXX------        
#             XXXXXX------        
#              XXXX ------        
# So the robot will turn ccw if the wall on its left is to far away, and right if it's to close. 
# It will also turn right if something is to close in the front.



# code that runs immediately when this file is executed:
if __name__ == "__main__":
	try:
		wall_follow()
	except rospy.ROSInterruptException:
		pass
