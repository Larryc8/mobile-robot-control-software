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
    points_state = pyqtSignal(
        str, str, int
    )  ## current pointid, next point id, numbres of points tha left, total points
    patrol_progress = pyqtSignal(float, int, int)

    def __init__(self, points=[], done_task=None, feedback_task=None) -> None:
        super().__init__()
        # rospy.init_node("action_client_move_base")
        # self._goals = [
        #     {"x_meters": 1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": 1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": -1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": -1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
        # ]
        self.current_patrolid = None
        self._goals = {}
        self.goals = {}
        # self.goals = self._goals.copy()
        self.done_task = done_task
        self.feedback_task = feedback_task
        self.cancelled = False
        self.emit_callback = None
        self.client = None
        # actionlib.GoalStatus.SUCCEEDED

    def setGoals(self):
        self.goals = self._goals.copy()

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

    def cancel_points_scheduling(self):
        if self.client:
            if self.client.get_state() == GoalStatus.ACTIVE:
                self.client.cancel_goal()
                self.cancelled = True
                # self.done_task()
                # self.restart()
        print('points canceled from points sche')

    def dispatch(self, patrolid =None):
        self.current_patrolid = float(patrolid)

        print("points points_scheduler points", self.goals)
        if len(self.goals) > 0:
            self.pointid, pose = self.goals.popitem()
            # ids_list = list(self.goals.keys())
            # if len(ids_list) < 2:
            #     self.points_state.emit(self.id, None, len(self.goals), len(self._goals))
            # else:
            #     self.points_state.emit(id, ids_list[-1], len(self.goals), len(self._goals))
            x_meters, y_meters, yaw_degrees, check = pose.values()
            goal = self.configGoal(x_meters, y_meters, yaw_degrees)
            self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            self.client.wait_for_server()
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def restart(self):
        self.goals = self._goals.copy()
        self.points_state.emit(None, None, 0)

    def setDoneTask(self, done_task):
        self.done_task = done_task

    def done_cb(self, state, result):
        ids_list = list(self.goals.keys())
        if len(ids_list) == 0:
            self.points_state.emit(self.pointid, None, state)
        else:
            self.points_state.emit(
                self.pointid,
                ids_list[-1],
                state,
            )

        self.patrol_progress.emit(self.current_patrolid, len(self.goals), len(self._goals))
        rospy.loginfo("Finished in state %s", str(state))
        if len(self.goals) == 0:
            if self.done_task:
                self.done_task()
            print("TODAS LOS PUNTOS HAN SIDO RECORRIDOS")
            return

        if self.cancelled:
            self.cancelled = False
            print('POINTS CANCELLED WITH state ', state )
            return

        self.dispatch(str(self.current_patrolid))

    def active_cb(self):
        rospy.loginfo("Goal just went active")
        self.patrol_progress.emit(self.current_patrolid, len(self.goals), len(self._goals))
        self.points_state.emit(self.pointid, self.pointid, 0)

    def feedback_cb(self, feedback):
        if self.feedback_task:
            self.feedback_task(feedback)
        # time.sleep(2)
        if self.client.get_state() == GoalStatus.ACTIVE:
            # self.cancel_goal()
            pass
        # if self.emit_callback:
            # self.emit_callback(str(self.current_patrolid), len(self.goals), len(self._goals))
        # self.patrol_progress.emit(str(self.current_patrolid), len(self.goals), len(self._goals))
        # rospy.loginfo(f"Got Feedback: {1}")

    # def patrol_progress(self,  callback):
        # self.emit_callback = callback#callback(str(self.current_patrolid), len(self.goals), len(self._goals))


    def update_points(self, points):
        self._goals = points
        print("from points_scheduler POINTS UPDATE")


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
