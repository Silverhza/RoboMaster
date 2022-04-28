#!/usr/bin/python
import rospy
import roslib
from roborts_msgs.msg import RobotShoot, ProjectileSupply, SupplierStatus, ShootInfo, GameStatus

class BulletCount(object):	
	def __init__(self, init):
		self.remain = init
		self.sent = 0
		self.last_fss = 0
		self.now_fss = 0
		self.add = 0
		self.status = 0

	def rs_cb(self, msg):
		if msg.speed > 0:
			self.sent += 1
			self.remain -= 1
			self.remain = max(0, self.remain)
    	
	def ps_cb(self, msg):
		self.add = msg.number

	def fss_cb(self, msg):
		#rospy.loginfo("fss_last: " + str(self.last_fss))
		#rospy.loginfo("fss_now: " + str(self.now_fss))
		self.last_fss = self.now_fss
		self.now_fss = msg.status
		if self.now_fss == 1:
			self.status = 2 #supplying
		if self.last_fss == 2 and self.now_fss == 0:
			self.status = 3 #finish
			self.remain += self.add
			self.add = 0

	def game_cb(self, msg):
		if msg.game_status == 4:
			if msg.remaining_time == 179 or msg.remaining_time == 119 or msg.remaining_time == 59:
				self.status = 1 # can supply

if __name__ == '__main__':
    rospy.init_node("BulletCounter", anonymous=True)
    init_num = rospy.get_param("~init_bullet", 0)

    bullet_pub = rospy.Publisher("/bullet", ShootInfo, queue_size=10)
    rate = rospy.Rate(10)

    counter = BulletCount(init_num) # init_bullet

    rospy.Subscriber("/robot_shoot", RobotShoot, counter.rs_cb)
    rospy.Subscriber("/projectile_supply", ProjectileSupply, counter.ps_cb)
    rospy.Subscriber("/field_supplier_status", SupplierStatus, counter.fss_cb)
    rospy.Subscriber("/game_status", GameStatus, counter.game_cb)

    while not rospy.is_shutdown():
        info = ShootInfo()
        info.remain_bullet = counter.remain
        info.sent_bullet = counter.sent
        info.supply_status = counter.status
        bullet_pub.publish(info)
        rate.sleep()
