#!/usr/bin/env python

import rospy
import random
from assignment1.srv import Target, TargetResponse

def random_target(request):
	"""	
	Function that generates two random coordinates (x,y) given the range
	(min,max)
	
	Args:
		request: The request of Target.srv
		- minimum value of the random coordinate
		- maximum value of the random coordinate
	
	Returns:
		Random point (x,y) between the minimum and maximum value
	"""
	# getting the target Response (target_x, target_y)
	result = TargetResponse()
	
	# choosing randomly the two coordinates of the target point
	result.target_x = random.uniform(request.minimum, request.maximum)
	result.target_y = random.uniform(request.minimum, request.maximum)
	
	# display the random target
	rospy.loginfo("Target position [%.2f,%.2f]",result.target_x,result.target_y)
	
	return result
	
def main():
	"""	
	Main function that init a node called target_server and create
	a service called target, used to generate a random point
	"""
	
	rospy.init_node('target_server') 
	
	# create a service called target, type Target,
	# using a function called random_target
	s = rospy.Service('target',Target,random_target)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
