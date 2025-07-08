#!/usr/bin/env python
import rospy
import time
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#from scipy.spatial.transform import Rotation
from scipy.linalg import norm
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ControlDynamicPose:
    def __init__(self, max_speed=None, max_turn=None):
        # rospy.init_node("imu_to_cmd_vel", anonymous=True)
        rospy.init_node("harold_start_launch", anonymous=True)

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

        # self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        rospy.loginfo("IMU to cmd_vel node started")

    def imu_callback(self, msg):
        # print(len(msg.ranges))
        self.distance = min(msg.ranges[:90] + msg.ranges[270:])
        self.cmd_vel.linear.x = self.max_linear_velocity * self.y
        self.cmd_vel.angular.z = self.max_angular_velocity * -self.x

        if self.stop:
            return 

        if (self.x == 0 and self.y == 0):
            self.stop = True 

        self.cmd_vel_pub.publish(self.cmd_vel)

    def setRobotDynamicPose(self, x, y):
        self.x = x
        self.y = y

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
        converter = ControlDynamicPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
