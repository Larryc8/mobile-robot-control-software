#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

# Import your action message, for example:
from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback
import datetime
# import asyncio

from PyQt5.QtCore import QThread, pyqtSignal, QObject #, pyqtSlot

class ScheduleChecker(QThread):
    progress_updated = pyqtSignal(int)
    task_completed = pyqtSignal(str)
    
    def __init__(self, task_id=1):
        super().__init__()
        self.task_id = task_id
        
    def run(self):
        try:
            for i in range(1, 101):
                # Simulate work
                time.sleep(0.05)
                
                # Update progress
                self.progress_updated.emit(i)
                
                # Check if thread should stop
                if self.isInterruptionRequested():
                    self.task_completed.emit(f"Task {self.task_id} cancelled")
                    return
                    
            # Task completed
            self.task_completed.emit(f"Task {self.task_id} finished successfully")
        except Exception as e:
            self.task_completed.emit(f"Task {self.task_id} failed: {str(e)}")

class PatrolsEscheduler(QObject):
    compelted_patrols = []
    looped_patrols = []
    schedule_checker = ScheduleChecker()

    def __init__(self, id=5, date=None):
        rospy.init_node("action_client_node", anonymous=True)
        self.client = actionlib.SimpleActionClient("patrols_server", PatrolAction)
        self.id = id
        self.date = date
        self.patrols = [
            {
                "id": 1,
                "days": ['Lun', 'Mar', 'Mier', 'Jue'],
                "time": '22:23',
                "delayed": False,
                "looped": False,
                "finished": False,
            }
        ]
        self.patrols_for_scheduling = self.patrols.copy()
        self.scheduled_patrols = []
        self.isScheduling = True
        self.isPatrolArrayModified = False

        # Wait for the action server to start
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server found!") 

    def schedule_all_patrols(self):
        pass

    def sort_patrols(self):
        pass

    def getTimeDifference(self, date, current_date):
        return 0

    def send_goal(self, patrol, done_callback=None):
        goal = PatrolGoal()
        goal.date = self.date
        goal.id = self.id

        if done_callback:
            done_cb = done_callback
        else:
            done_cb = self.done_callback

        self.client.send_goal(
            goal,
            done_cb=done_cb,
            active_cb=self.active_callback,
            feedback_cb=self.feedback_callback,
        )

        rospy.loginfo("Goal sent!")


    def active_callback(self):
        rospy.loginfo("Goal is now active")

    def feedback_callback(self, feedback):
        rospy.loginfo("Received feedback: {} complete".format(feedback.points_checked))

    def done_callback(self, state, result):
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded! Result: {}")
            rospy.loginfo(self.client.get_goal_status_text())
        else:
            rospy.loginfo(self.client.get_goal_status_text())
            rospy.logerr("somthing just happends!!")

    def cancel_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled")


if __name__ == "__main__":
    try:
        client = PatrolsEscheduler()
        # result = client.send_goal()
        # rospy.loginfo("Final result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
