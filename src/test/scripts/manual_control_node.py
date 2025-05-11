#!/usr/bin/env python
import rospy
import time
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation
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
        self.x = 0
        self.y = 0

        # self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

        rospy.loginfo("IMU to cmd_vel node started")

    def imu_callback(self, msg):
        print(len(msg.ranges))
        self.distance = min(msg.ranges[:90]  + msg.ranges[270:])
        # self.distance = msg.ranges[0]
        # if distance < self.safe_distance:
        self.cmd_vel.linear.x = self.max_linear_velocity * self.y
        self.cmd_vel.angular.z = self.max_angular_velocity * -self.x

        self.cmd_vel_pub.publish(self.cmd_vel)
        print('x y joy> ', self.x, self.y, self.distance)


        # cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        # orientation = odom.pose.pose.orientation
        #     # (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # (roll, pitch, theta) = euler_from_quaternion(
        #     [orientation.x, orientation.y, orientation.z, orientation.w]
        # )
        # # self.yaw = quaternion_orientation.as_euler("xyz", degrees=True)[2]

        # x = self.x
        # y = self.y
        # target_yaw = math.atan2(x, y)
        # deg_target_yaw = math.degrees(target_yaw)
        # anglerobot = math.degrees(theta) % 360 
        # target = deg_target_yaw%360 
        # print('el anguo odom dd target', math.degrees(theta) % 360,  deg_target_yaw%360)
        # target_linear_velocity = norm([x, y]) * self.max_linear_velocity
        # # inc_x = x - 1*math.cos(self.yaw)
        # # inc_y = y - 1*math.sin(self.yaw)
        # # angle_to_goal = math.atan2(inc_y, inc_x)

        # # print("yaw 1.robotyaw 2.targetyaw", quaternion_orientation.as_euler("xyz", degrees=True), round(deg_target_yaw))
        # # print('x y', x, y)
        # if not (x == 0 and y == 0):
        #     if abs(target - anglerobot) > 12:
        #         self.cmd_vel.angular.z = self.max_angular_velocity*(target - anglerobot)*4/360
        #         self.cmd_vel.linear.x = 0 
        #         # time.sleep(1)
        #     else:
        #         self.cmd_vel.angular.z = 0 
        #         self.cmd_vel.linear.x = target_linear_velocity 

        #     # time.sleep(1)
        #     cmd_vel_pub.publish(self.cmd_vel)
        # else:
        #     # time.sleep(1)
        #     self.cmd_vel.linear.x = 0 
        #     self.cmd_vel.angular.z = 0
        #     cmd_vel_pub.publish(self.cmd_vel)
        pass


    def setRobotDynamicPose(self, x, y ):
        self.x = x
        self.y = y
 


        # target_yaw = math.atan2(x, y)
        # deg_target_yaw = math.degrees(target_yaw)
        # target_linear_velocity = norm([x, y]) * self.max_linear_velocity
        # inc_x = x - 1*math.cos(self.yaw)
        # inc_y = y - 1*math.sin(self.yaw)
        # angle_to_goal = math.atan2(inc_x, inc_y)

        # print("yaw 1.robotyaw 2.targetyaw", round(self.yaw), round(deg_target_yaw))
        # print('x y', x, y)
        # if not (x == 0 and y == 0):
        #     self.cmd_vel.linear.x = target_linear_velocity
        #     self.cmd_vel.angular.z = self.max_angular_velocity*(target_yaw - self.yaw)/200
        #     self.cmd_vel_pub.publish(self.cmd_vel)
        # else:
        #     self.cmd_vel.linear.x = 0 
        #     self.cmd_vel.angular.z = 0
        #     self.cmd_vel_pub.publish(self.cmd_vel)
    # def continuousAngle(self, angle_deg):
    #     if angle_deg <:
    #         angle = 360 - angle_deg
    #     if angle_deg > 0:
    #         angle = 360 - angle_deg
    #     if angle_deg > 0:
    #         angle = 360 - angle_deg
    #     if angle_deg > 0:
    #         angle = 360 - angle_deg
    #     if angle_deg < 0:
    #         angle = 
    #     
    #     return angle_deg




if __name__ == "__main__":
    try:
        converter = ControlDynamicPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
