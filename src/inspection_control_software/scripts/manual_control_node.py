#!/usr/bin/env python
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QObject, QRect, QSize, Qt, QThread, QTimer, pyqtSignal

# from scipy.spatial.transform import Rotation
from scipy.linalg import norm
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion


class DynamicPoseController(QObject):
    def __init__(self, max_speed=None, max_turn=None):
        super().__init__()
        # rospy.init_node("imu_to_cmd_vel", anonymous=True)
        # rospy.init_node("harold_start_launch", anonymous=True)

        self.max_linear_velocity = 0.1  # m/s
        self.max_angular_velocity = 0.3  # rad/s
        self.cmd_vel = Twist()
        self.target_yaw_error = 5
        self.safe_distance = 0.4
        self.distance = 10
        self.isManualControl = False
        self.x = 0
        self.y = 0
        self.stop = False

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.vel_control_callback)
        self.timer.start(30)  # Update at ~30fps

        # self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.vel_control_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        rospy.loginfo("IMU to cmd_vel node started")

    def vel_control_callback(self, msg=None):
        # print(len(msg.ranges))
        #
        if not rospy.has_param("/max_linear_velocity"):
            return

        if not rospy.has_param("/max_angular_velocity"):
            return

        self.max_linear_velocity = rospy.get_param("/max_linear_velocity")
        self.max_angular_velocity = rospy.get_param("/max_angular_velocity")

        # self.distance = min(msg.ranges[:90] + msg.ranges[270:])
        self.cmd_vel.linear.x = self.max_linear_velocity * self.y
        self.cmd_vel.angular.z = self.max_angular_velocity * -self.x

        if self.stop:
            return

        if self.x == 0 and self.y == 0:
            self.stop = True

        self.cmd_vel_pub.publish(self.cmd_vel)

    def setRobotDynamicPose(self, x, y):
        self.x = x
        self.y = y

        # self.cmd_vel.linear.x = self.max_linear_velocity * self.y
        # self.cmd_vel.angular.z = self.max_angular_velocity * -self.x
        # self.cmd_vel_pub.publish(self.cmd_vel)
        if not (self.x == 0 and self.y == 0):
            self.stop = False

    def setControlMode(self, mode):
        if mode == "manual":
            self.isManualControl = True
            return

        if mode == "auto":
            self.isManualControl = False
            return
        raise ValueError("specify a valid operation mode [auto, manual]")


if __name__ == "__main__":
    try:
        converter = DynamicPoseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
