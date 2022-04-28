import rospy
from roborts_msgs.msg import TwistAccel
from roborts_msgs.msg import GimbalAngle

def talker():
    
    pub = rospy.Publisher('/cmd_gimbal_angle', GimbalAngle, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    gimbleCmd = GimbalAngle()
    rate = rospy.Rate(1) 
    
    
    while not rospy.is_shutdown():
        gimbleCmd.yaw_mode = false;
        gimbleCmd.pitch_mode = false;
        gimbleCmd.yaw_angle = 0.5;
        gimbleCmd.pitch_angle = 0;
        theta
        pub.publish(gimbleCmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        talker()
    except rospy.ROSInterruptException: pass
