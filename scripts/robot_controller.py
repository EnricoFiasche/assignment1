#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from assignment1.srv import Target
from geometry_msgs.msg import Twist

## Global publisher to modify the velocity inside the callBack function
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

## Client service that generate a random target (x,y)
random_target_client = rospy.ServiceProxy('target',Target)

## Target position coordinates
target = Odometry()

def positionCallBack(robot_position):
	"""
	CallBack function that reads the actual position of the robot and
	checks if it reached the target.
	If it is done, generates another target point and modify its
	velocity.
	If the target isn't reached modify its velocity based by the
	distance between the robot and target
	
	Args:
		robot position: The robot position given by the topic odom
	"""

	x = robot_position.pose.pose.position.x # x coordinate of robot
	y = robot_position.pose.pose.position.y # y coordinate of robot
	
	if(isReached(x,y)): #if the target is reached
		# call for another random target
		response = random_target_client(-6.0,6.0)
		target.pose.pose.position.x = response.target_x
		target.pose.pose.position.y = response.target_y
	
	# setting the velocity of the robot based by the distance
	# between the target and robot position
	vel = Twist()
	vel.linear.x = (target.pose.pose.position.x - x)
	vel.linear.y = (target.pose.pose.position.y - y)
		
	pub.publish(vel)
		
# function that checks if the position is reached 
def isReached(x,y):
	"""
	Function that checks if the robot has reached the taget position,
	comparing its position with the target, with an error of 0.1
	
	Args:
		x: The robot position along x-axis
		y: The robot position along y-axis
		
	Returns:
		Boolean value that indicates if the target is reached by the
		robot.
	"""
	
	# if the x position is in the range +-0.1
	if(x >= target.pose.pose.position.x - 0.1 and x <= target.pose.pose.position.x + 0.1):
		# if the y position is in the range +-0.1
		if(y >= target.pose.pose.position.y - 0.1 and y <= target.pose.pose.position.y + 0.1):
			return True # the target is reached
	
	return False # the target is not reached
	
def control():
	"""
	Main function that generates a node called robot_controller.
	It generates a random target coordinates and create a subscriber
	to know the robot position
	"""
	
	rospy.init_node('robot_controller')

	response = random_target_client(-6.0,6.0)
	target.pose.pose.position.x = response.target_x
	target.pose.pose.position.y = response.target_y
    
    # subscribers that callBack the robot position
	rospy.Subscriber("/odom", Odometry, positionCallBack)
    
	rospy.spin()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
