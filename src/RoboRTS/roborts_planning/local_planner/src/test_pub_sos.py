#!/usr/bin/python
import rospy
import os
#import geometry_msgs.msg
from nav_msgs.msg import Path
from actionlib import SimpleActionClient
#from roborts_msgs.msg import LocalPlannerAction
import roborts_msgs.msg
from std_msgs.msg import Empty


if __name__ == '__main__':
	
	rospy.init_node('my_pcl_test_node', anonymous=True)

	CAR_ID = os.environ.get('CAR_ID')

	sos_pub = rospy.Publisher("/" + CAR_ID + "/global_planner_sos", Empty, queue_size=10)
	

	# Empty help_
	while True:
		sos_pub.publish(Empty())
