#!/usr/bin/env python

import numpy as np
import sys
import math
import logging
import time


from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


from alert_generator import AlertGenerator
from database_manager import AlertStatus


# logging.basicConfig(filename='app.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
file_handler = logging.FileHandler("app.log")
file_handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)
# logging.basicConfig(
#     filename='navigation_checker.log',        # Name of the log file
#     level=logging.INFO,        # Minimum level to log
# )


def quadratic_median_error(data1, data2):
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

    squared_diffx = [e for i, e in enumerate(ax) if (ax[i] - bx[i]) ** 2 > 0.06]
    squared_diffy = [e for i, e in enumerate(ay) if (ay[i] - by[i]) ** 2 > 0.06]

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
        self.checkpoint_id = None
        self.patrol_id = None
        self.map = None

    def set_current_goal(self, goal, id):
        print("set current goal navigation checker", goal)
        self.goal = goal
        self.num = id

    def callback_globalplan(self, data):
        if not self.old_plan:
            self.old_plan = data.poses
        else:
            current_path = list(data.poses)
            initial_path = list(self.old_plan)

            current_path_len = len(current_path)
            initial_path_len = len(initial_path)

            if current_path_len == initial_path_len:
                error = quadratic_median_error(current_path, initial_path)
                # print("match", error)
            else:
                min_len = min(current_path_len, initial_path_len)
                current_path = current_path[current_path_len - min_len :]
                initial_path = initial_path[initial_path_len - min_len :]
                error = quadratic_median_error(current_path, initial_path)
            self.old_plan = data.poses

            likehood = 100 - error * 100 / current_path_len
            if likehood < 80 and self.checkpoint_id:
                self.alert.throw_alert(
                    "obstacle", AlertStatus.ERROR.value,
                    self.patrol_id, self.checkpoint_id, self.map
                )
                self.alert_generated.emit('Se detecto un obtaculo!!', AlertStatus.ERROR)
    
            print(f"dont match, {likehood} {self.track}")
            logger.error(f"dont match, {likehood}, {self.track}")

    def start_checker(self, patrol_id, checkpoint_id, map):
        self.old_plan = None
        self.checkpoint_id = checkpoint_id
        self.patrol_id = patrol_id
        self.map = map

        self.listen()

    def listen(self):
        self.sub_globalplan = rospy.Subscriber(
            "/move_base/NavfnROS/plan", Path, self.callback_globalplan
        )

class StuckDetector:
    def __init__(self):
        # Variables to store latest data
        self.current_velocity = 0.0
        self.commanded_velocity = 0.0
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
