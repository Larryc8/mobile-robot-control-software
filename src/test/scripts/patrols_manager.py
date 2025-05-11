#!/usr/bin/env python

import rospy
import actionlib
from test.msg import (
    PatrolAction,
    PatrolGoal,
    PatrolResult,
    PatrolFeedback,
)
from geometry_msgs.msg import Pose
# import asyncio
# import datetime

class PatrolsServer:
    def __init__(self):
        rospy.init_node("patrol_server")
        self.server = actionlib.SimpleActionServer(
            "patrols_server", PatrolAction, self.execute, False
        )
        self.server.start()
        rospy.loginfo("Countdown Action Server started")


    def execute(self, goal):
        # Check if the goal is valid
        date = goal.date
        id = goal.id
        if len(date) <= 1:
            result = PatrolResult()
            self.server.set_aborted(result, "String vacio en date")
            return

        feedback = PatrolFeedback()

        for i in range(id, -1, -1): ## this for loop rpresent every point covered
            # Check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("Goal preempted")
                result = PatrolResult()
                self.server.set_preempted(result)
                return

            feedback.points_checked = i
            self.server.publish_feedback(feedback)

            # Sleep for demonstration purposes
            rospy.sleep(1.0)

        # Return the result when done
        result = PatrolResult()
        pose = Pose()
        pose.position.x = 1
        pose.position.y = 1
        pose.position.z = 1
        result.points_pose_array = [pose]
        print(result)
        self.server.set_succeeded(result, "Todo bien, dicel el pibe")


if __name__ == "__main__":
    # rospy.init_node("countdown_server")
    server = PatrolsServer()
    rospy.spin()
