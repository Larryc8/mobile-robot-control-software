#!/usr/bin/env python

import logging
import math
import sys
import time

import numpy as np
import rospy
from alert_generator import AlertGenerator
from database_manager import AlertStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from PyQt5.QtCore import QObject, QSettings, QThread, pyqtSignal
from sensor_msgs.msg import Imu
from sklearn.utils.extmath import squared_norm
from std_msgs.msg import Bool


def quadratic_median_error(data1, data2, tolerance=0.06):
    ax = []
    ay = []
    bx = []
    by = []

    for d in data1:
        x = d.pose.position.x
        y = d.pose.position.y
        ax.append(x)
        ay.append(y)

    for d in data2:
        x = d.pose.position.x
        y = d.pose.position.y
        bx.append(x)
        by.append(y)

    ax = np.array(ax)
    ay = np.array(ay)
    bx = np.array(bx)
    by = np.array(by)

    squared_diffx = [e for i, e in enumerate(ax) if (ax[i] - bx[i]) ** 2 > tolerance]
    squared_diffy = [e for i, e in enumerate(ay) if (ay[i] - by[i]) ** 2 > tolerance]

    squared_meanx = np.mean([(ax[i] - bx[i]) ** 2 for i, e in enumerate(ax)])
    squared_meany = np.mean([(ay[i] - by[i]) ** 2 for i, e in enumerate(ay)])

    squared_mean = squared_meanx + squared_meany

    return max(len(squared_diffy), len(squared_diffx))


class RobotNavigationChecker(QObject):
    alert_generated = pyqtSignal(str, AlertStatus)

    def __init__(self, track) -> None:
        super().__init__()
        self.sub_globalplan = None

        self.current_pose = None
        self.saved_pose = None
        self.global_plan = None
        self.old_plan = None
        self.track = track
        self.alert = AlertGenerator()
        self._checkpoint_id = None
        self._patrol_id = None
        self.map = None
        self.TOLERANCE = 0.06
        self._settings = QSettings("MyCompany", "MyApp")
        self.stuck_checker = StuckDetector()

    def set_current_goal(self, goal, id):
        print("set current goal navigation checker", goal)
        self.goal = goal
        self.num = id

    def callback_globalplan(self, data):
        tolerance: float = self._settings.value("pathTolerance", self.TOLERANCE)

        if not self.old_plan:
            self.old_plan = data.poses
        else:
            current_path = list(data.poses)
            initial_path = list(self.old_plan)

            current_path_len = len(current_path)
            initial_path_len = len(initial_path)

            if current_path_len == initial_path_len:
                error = quadratic_median_error(current_path, initial_path, tolerance)
            else:
                min_len = min(current_path_len, initial_path_len)
                current_path = current_path[current_path_len - min_len :]
                initial_path = initial_path[initial_path_len - min_len :]
                error = quadratic_median_error(current_path, initial_path)
            self.old_plan = data.poses

            likehood = 100 - error * 100 / current_path_len

            if likehood < 80 and self._checkpoint_id:
                self.alert.throw_alert(
                    "obstacle",
                    AlertStatus.ERROR.value,
                    self._patrol_id,
                    self._checkpoint_id,
                    self.map,
                )
                self.alert_generated.emit("Se detecto un obtaculo!!", AlertStatus.ERROR)

            print(f"dont match, {likehood} {self.track}")
            # logger.error(f"dont match, {likehood}, {self.track}")

    def start_checker(self, patrol_id, checkpoint_id, map):
        self.old_plan = None
        self._checkpoint_id = checkpoint_id
        self._patrol_id = patrol_id
        self.map = map
        # self.TOLERANCE = rospy.get_param("/path_tolerance")

        self.listen()

    def listen(self):
        self.sub_globalplan = rospy.Subscriber(
            "/move_base/NavfnROS/plan", Path, self.callback_globalplan
        )


class StuckDetector:
    def __init__(self):
        # rospy.init_node("stuck_detector_node", anonymous=True)

        # --- Thresholds & Parameters ---
        self.cmd_thresh = rospy.get_param(
            "~cmd_thresh", 0.05
        )  # Minimum command to consider "trying to move"
        self.odom_thresh = rospy.get_param(
            "~odom_thresh", 0.01
        )  # Maximum odom speed to consider "stopped"
        self.stuck_timeout = rospy.get_param(
            "~stuck_timeout", 3.0
        )  # Seconds before declaring stuck

        # --- State Variables ---
        self.last_cmd_time = rospy.Time.now()
        self.is_commanding_movement = False
        self.is_actually_moving = False
        self.stuck_start_time = None
        self.is_stuck = False

        # --- Publishers & Subscribers ---
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.stuck_pub = rospy.Publisher("/robot_stuck", Bool, queue_size=10)

        # --- Timer ---
        # Checks the status at 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_stuck_status)

        rospy.loginfo("Stuck Detector Node Initialized.")

    def cmd_callback(self, msg):
        # Calculate magnitude of linear and angular commands
        linear_cmd = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        angular_cmd = abs(msg.angular.z)

        # Check if the navigation system is actively trying to move the robot
        if linear_cmd > self.cmd_thresh or angular_cmd > self.cmd_thresh:
            self.is_commanding_movement = True
            self.last_cmd_time = rospy.Time.now()
        else:
            # If we haven't received a move command recently, we aren't trying to move
            if (rospy.Time.now() - self.last_cmd_time).to_sec() > 0.5:
                self.is_commanding_movement = False

    def odom_callback(self, msg):
        # Calculate actual linear and angular speed from Odometry
        linear_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        angular_speed = abs(msg.twist.twist.angular.z)

        # Check if the robot is physically moving
        if linear_speed > self.odom_thresh or angular_speed > self.odom_thresh:
            self.is_actually_moving = True
        else:
            self.is_actually_moving = False

    def check_stuck_status(self, event):
        # Condition for being stuck: Command sent, but no physical movement
        if self.is_commanding_movement and not self.is_actually_moving:
            if self.stuck_start_time is None:
                self.stuck_start_time = rospy.Time.now()
            else:
                elapsed_time = (rospy.Time.now() - self.stuck_start_time).to_sec()
                if elapsed_time >= self.stuck_timeout:
                    if not self.is_stuck:
                        self.is_stuck = True
                        rospy.logwarn(
                            "ROBOT IS STUCK! Command sent but no movement for {}s".format(
                                elapsed_time
                            )
                        )
        else:
            # Reset if the robot starts moving or stops receiving commands
            self.stuck_start_time = None
            if self.is_stuck:
                self.is_stuck = False
                rospy.loginfo("Robot is free and moving again.")

        # Publish the stuck status
        self.stuck_pub.publish(Bool(self.is_stuck))
