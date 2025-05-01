#!/usr/bin/env python
from typing import Callable
from typing import Any

import sys
import rospy
import actionlib
from scipy.spatial.transform import Rotation

from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
    MoveBaseFeedback,
)


class PointsScheduler:
    def __init__(
            self, done_task = None, feedback_task = None
    ) -> None:
        rospy.init_node("action_client_move_base")
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

    def cancel(self):
        self.client.cancel_goal()
        self.restart()

    def dispatch(self):
        pose = self.goals.pop()
        goal = self.configGoal(pose["x_meters"], pose["y_meters"], pose["yaw_degrees"])
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def restart(self):
        self.goals = self.goals.copy()

    def done_cb(self, state, result):
        rospy.loginfo("Finished in state %s", str(state))
        if len(self.goals) == 0:
            if self.done_task:
                self.done_task()
            print("TODAS LOS PUNTOS HAN SIDO RECORRIDOS")
            # sys.exit(0)
            # callable
            return
        self.dispatch()

    def active_cb(self):
        rospy.loginfo("Goal just went active")

    def feedback_cb(self, feedback):
        if self.feedback_task:
            self.feedback_task()
        # rospy.loginfo(f"Got Feedback: {1}")
        pass


# Can do other work here
if __name__ == "__main__":
    def cbdone():
        print("soy un callback de DONE")
    def cbfeeback():
        print('SOy un callback de FEEDBACK')

    ex = PointsScheduler(done_task=cbdone, feedback_task=cbfeeback)
    ex.dispatch()
    rospy.spin()
