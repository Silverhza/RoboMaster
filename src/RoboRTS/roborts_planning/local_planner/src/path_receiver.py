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

class send2localPlanner:
    def callback(self,path):
        self.local_planner_goal_ = roborts_msgs.msg.LocalPlannerGoal()
        self.local_planner_goal_.route = path
        # print(self.local_planner_goal_)
        self.vis_pub.publish(path)
        self.local_planner_client_.send_goal(self.local_planner_goal_)

    def pose_callback(self,pose):
        self.pose_ = pose;
        # print("POSE GOT")

    def __init__(self):
        self.local_planner_client_ = SimpleActionClient('/'+CAR_ID+'/local_planner_node_action', roborts_msgs.msg.LocalPlannerAction)
        rospy.Subscriber("/"+CAR_ID+"/decision_global_path", Path, self.callback)
        rospy.Subscriber("/"+CAR_ID+"/amcl_pose", PoseStamped, self.pose_callback)
        self.vis_pub = rospy.Publisher("/"+CAR_ID+"/global_planner_node/path", Path, queue_size=10)
        #print('before wait')
        self.local_planner_client_.wait_for_server()
        print('[PATH RECEIVER] INITIALISED IN ' + CAR_ID +' !')

def thread_job():
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('path_receiver_node', anonymous=True)

    CAR_ID = os.environ.get('CAR_ID')


    sender = send2localPlanner()

    rospy.spin()
