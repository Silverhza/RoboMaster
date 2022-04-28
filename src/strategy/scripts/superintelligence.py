import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String
import a_star
import matplotlib.pyplot as plt
import numpy as np
from math import ceil
import time
import math
from scipy.spatial.transform import Rotation as R
import threading
import utility


class Robot:
    x, y, yaw = 0.0, 0.0, 0.0
    theta = 0.25
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
    def __str__(self):
        return "x: %.2f, y: %.2f" % (self.x, self.y)
    def __eq__(self, other):
        return abs(self.x - other.x) < self.theta and abs(self.y - other.y) < self.theta

    def __sub__(self, other):
        return self.x - other.x, self.y - other.y

class Brain:
    def __init__(self):
        self._show_animation = True
        self._resolution = 0.1
        self.w, self.h = 8.1, 5.1
        self.width, self.height = int(ceil(self.w / self._resolution)), int(ceil(self.h / self._resolution))

        self._map = self.initMap()
        self._decision_pubs = [rospy.Publisher("/CAR1/move_base_simple/goal", PoseStamped, queue_size=10),
                              rospy.Publisher("/CAR2/move_base_simple/goal", PoseStamped, queue_size=10)]
        self._debuff_subscriber = rospy.Subscriber("/debuff", String, self.receiveDebuffSignal)
        self._robots_subscribers = [rospy.Subscriber("/CAR1/amcl_pose", PoseStamped, self.car1PositionCB),
                                   rospy.Subscriber("/CAR2/amcl_pose", PoseStamped, self.car2PositionCB)]
        self._enemies_subscriber = rospy.Subscriber("/CAR1/obstacle_filtered", Obstacles, self.enemyCB) # TODO: FROM ShaoGang

        self._debuff = []
        self.robots = [Robot(0.0, 0.0, 0.0), Robot(0.0, 0.0, 0.0)]
        self.enemies = []
        self._rho = 1.2
        self._rangeAngle = 5
        self._attackDist = 1.8
        # self._a_star_planner_class = self._planner_def_inner_class()
        # self._a_star_planner = self._a_star_planner_class()
        #
        # time_start = time.time()
        # rx, ry = self._a_star_planner.planning(sx=2, sy=2,
        #                                  gx=20, gy=20)
        # plt.plot(rx, ry, "r")
        # plt.show()

    # Problem 给定的随机点不一定是可达点
    def callFriend(self):
        if not self.observation:
            return

        goal = PoseStamped()
        goal.header.frame_id = "/map"
        target = self.observation[0]
        for i in range(2):
            yaw_angle = math.atan2(target.y - self.robots[i].y, target.x - self.robots[i].x)

            theta = 0
            if yaw_angle >= 0:
                theta = yaw_angle - math.pi
            elif yaw_angle < 0:
                theta = math.pi + yaw_angle
            else:
                rospy.logerr("invalid yaw")

            theta = theta + np.random.uniform(-self._rangeAngle / 180 * math.pi, self._rangeAngle / 180 * math.pi)
            theta = np.clip(theta, -math.pi, math.pi)
            target.x, target.y = target.x + self._rho * math.cos(theta), target.y + self._rho * math.sin(theta)

            goal.pose.position.x, goal.pose.position.y = target.x, target.y
            [goal.pose.orientation.w,
             goal.pose.orientation.x,
             goal.pose.orientation.y,
             goal.pose.orientation.z] = self._createQuaternionFromYaw(yaw_angle)

            self._decision_pubs[i].publish(goal)

    def isInDanger(self):
        if len(self.observation) < 2:
            return False
        robot0, robot1 = np.array(self.robots[0]), np.array(self.robots[1])
        enemy0, enemy1 = np.array(self.observation[0]), np.array(self.observation[1])

        flag = False
        if np.linalg.norm(robot0 - enemy0) < self._attackDist and np.linalg.norm(robot0 - enemy1) < self._attackDist:
            goal = self.selectGoal(self.robots[0].x, self.robots[0].y)
            self.escape(0, goal)
            flag = True

        if np.linalg.norm(robot1 - enemy0) < self._attackDist and np.linalg.norm(robot1 - enemy1) < self._attackDist:
            goal = self.selectGoal(self.robots[1].x, self.robots[1].y)
            self.escape(1, goal)
            flag = True

        return flag

    def selectGoal(self, x, y):
        areaID = utility.belongToDistrict(x, y)
        goalID = 7 - areaID
        goal = PoseStamped()
        goal.header.frame_id = "/map"
        goal.pose.position.x, goal.pose.position.y = utility.setGoal(goalID)
        goal.pose.orientation.w = 1
        print("now: ", areaID, "goal: ",goalID)
        return goal


    def escape(self, id, goal):
        self._decision_pubs[id].publish(goal)

    def _createQuaternionFromYaw(self, yaw):
        # input: r p y
        r = R.from_euler('zyx', [0, 0, yaw], degrees=False).as_quat()
        # output: w x y z
        return [r[3], r[2], r[1], r[0]]

    def initMap(self):
        map = np.zeros( (self.width, self.height), dtype = int)
        map[0, :], map[-1, :], map[:, 0], map[:, -1] = 1, 1, 1, 1

        v_obsticle = np.array([[0, 3.85], [3.6, 1],
                               [3.5, 3.85], [7.1, 1]])
        v_min_obsticle = np.array([[1.5, 2.425],
                                   [5.8, 2.425]])
        h_obsticle = np.array([[1.5, 0], [8.1 - 1.5 - 0.25, 4.1]])
        c_obsticle = np.array([[4, 2.4]])

        for i in v_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(1 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in v_min_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.8 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in h_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.25 / self._resolution))):
                for k in range(int(ceil(1 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in c_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.25 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        return map

    def draw(self):
        plt.imshow(self._map)
        plt.colorbar()
        plt.show()

    def receiveDebuffSignal(self, new_buff):
        self._debuff = []
        for i in range(1,7):
            if new_buff.data[2*i] == '1':
                self._debuff.append(i)

    

    def car1PositionCB(self, msg):
        self.robots[0].x = msg.pose.position.x
        self.robots[0].y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robots[0].yaw = y

    def car2PositionCB(self, msg):
        self.robots[1].x = msg.pose.position.x
        self.robots[1].y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robots[1].yaw = y

    def enemyCB(self, data):
        self.enemies = []
        for enemy in data.circles:
            self.enemies.append(Robot(enemy.center.x, enemy.center.y, 0.0))



def call_rosspin():
    rospy.spin()

if __name__ == '__main__':

    try:
        # making_decision()
        rospy.init_node('listener', anonymous=True)
        rate = rospy.Rate(1)
        brain = Brain()
        spin_thread = threading.Thread(target=call_rosspin).start()
        # rospy.spin()

        while not rospy.core.is_shutdown():
            # self.makeDecision()
            # brain.callFriend()
            # brain.isInDanger()
            print("============ROBO1=============")
            print(robots[0])
            print("============ROBO2=============")
            print(robots[1])
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
