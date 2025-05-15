#!/usr/bin/env python
from typing import Callable
from typing import Any

import sys
import rospy
import actionlib
from scipy.spatial.transform import Rotation
import time

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
    MoveBaseFeedback,
)

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot


class PointsScheduler(QObject):
    def __init__(self, points=[], done_task=None, feedback_task=None) -> None:
        super().__init__()
        # rospy.init_node("action_client_move_base")
        self._goals = [
            {"x_meters": 1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
            {"x_meters": 1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
            {"x_meters": -1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
            {"x_meters": -1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
        ]
        self.goals = self._goals.copy()
        self.done_task = done_task
        self.feedback_task = feedback_task
        # actionlib.GoalStatus.SUCCEEDED

    def mapPointsToGoals(self):
        pass

    def configGoal(self, x, y, yaw):
        quaternion = Rotation.from_euler("z", yaw, True).as_quat()
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        return goal

    def setHomePoint(self):
        pass

    def cancel_goal(self):
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_goal()
            # self.done_task()
            # self.restart()

    def dispatch(self):
        if len(self.goals) > 0:
            pose = self.goals.pop()
            goal = self.configGoal(pose["x_meters"], pose["y_meters"], pose["yaw_degrees"])
            self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            self.client.wait_for_server()
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def restart(self):
        self.goals = self._goals.copy()

    def setDoneTask(self, done_task):
        self.done_task = done_task

    def done_cb(self, state, result):
        rospy.loginfo("Finished in state %s", str(state))
        if len(self.goals) == 0:
            if self.done_task:
                self.done_task()
            print("TODAS LOS PUNTOS HAN SIDO RECORRIDOS")
            return
        self.dispatch()

    def active_cb(self):
        rospy.loginfo("Goal just went active")

    def feedback_cb(self, feedback):
        if self.feedback_task:
            self.feedback_task(feedback)
        # time.sleep(2)
        if self.client.get_state() == GoalStatus.ACTIVE:
            # self.cancel_goal()
            pass
        # rospy.loginfo(f"Got Feedback: {1}")
    def update_points(self):
        print('from points_scheduler POINTS UPDATE')


# Can do other work here
if __name__ == "__main__":

    def cbdone():
        print("soy un callback de DONE")

    def cbfeeback(x):
        print("SOy un callback de FEEDBACK")
        print(x)

    ex = PointsScheduler(done_task=cbdone, feedback_task=cbfeeback)
    ex.dispatch()
    # ex.restart()
    # ex.dispatch()
    rospy.spin()
