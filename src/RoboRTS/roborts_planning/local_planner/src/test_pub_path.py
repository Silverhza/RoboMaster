#!/usr/bin/python
import rospy
import os
#import geometry_msgs.msg
from nav_msgs.msg import Path
from actionlib import SimpleActionClient
#from roborts_msgs.msg import LocalPlannerAction
import roborts_msgs.msg
from geometry_msgs.msg import PoseStamped
import threading

pose_ = None
pose_got = False

def pose_callback(pose):
	pose_ = pose;
	print("POSE GOT")
	pose_got = True
	path = Path()
	path.header.frame_id = "map"
	path.header.seq = 0

	for i in range(0,30):
		p = PoseStamped()
		p.header.frame_id = "map"

		p.pose.position.x = pose_.pose.position.x - i * 0.1
		p.pose.position.y = pose_.pose.position.y - i * 0.1
		p.pose.orientation.x = 0
		p.pose.orientation.y = 0
		p.pose.orientation.z = 0
		p.pose.orientation.w = 1
		path.poses.append(p);



	decision_pub.publish(path)

def thread_job():
	rospy.spin()


if __name__ == '__main__':
	
	rospy.init_node('my_pcl_test_node', anonymous=True)

	CAR_ID = os.environ.get('CAR_ID')

	decision_pub = rospy.Publisher("/" + CAR_ID + "/decision_global_path", Path, queue_size=10)
	rospy.Subscriber("/" + CAR_ID + "/amcl_pose", PoseStamped, pose_callback)

	rospy.spin()