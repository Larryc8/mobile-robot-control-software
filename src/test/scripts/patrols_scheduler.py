#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

# Import your action message, for example:
from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback
import datetime
# import asyncio

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot


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
    update_patrols_view = pyqtSignal(dict)

    def __init__(self, id=5, date=None):
        super().__init__()
        # rospy.init_node("action_client_node", anonymous=True)
        # self.client = actionlib.SimpleActionClient("patrols_server", PatrolAction)
        self.id = id
        self.date = date
        self.patrols = []

        self.patrols_data = {
            "0": {
                "days": {
                    "Lun": {"day": "Lun", "time": 20202, "finished": False},
                    "Mar": {"day": "Mar", "time": 20202, "finished": False},
                    "Mie": {"day": "Mar", "time": 20202, "finished": False},
                }, 
                "time": 200, 
                "state": 'encurso'
            },
            "1": {
                "days": {
                    "Lun": {"day": "Lun", "time": 20202, "finished": False},
                    "Mar": {"day": "Mar", "time": 20202, "finished": False},
                    "Mie": {"day": "Mar", "time": 20202, "finished": False},
                }, 
                "time": 200, 
                "state": 'encurso'
            },

        }
        # self.patrols = [
        #     {
        #         "id": 1,
        #         "days": 'Lun',
        #         "time": '2223',
        #         "delayed": False,
        #         "looped": False,
        #         "finished": False,
        #     },
        #    {
        #         "id": 1,
        #         "days": 'Vir',
        #         "time": '2220',
        #         "delayed": False,
        #         "looped": False,
        #         "finished": False,
        #     },
        #     {
        #         "id": 2,
        #         "days": 'Jue',
        #         "time": '2221',
        #         "delayed": False,
        #         "looped": False,
        #         "finished": False,
        #     },
        #     {
        #         "id": 2,
        #         "days": 'Mie',
        #         "time": '2224',
        #         "delayed": False,
        #         "looped": False,
        #         "finished": False,
        #     }
        # ]
        self.patrols_for_scheduling = self.patrols.copy()
        self.scheduled_patrols = []
        self.isScheduling = True
        self.isPatrolArrayModified = False

        # Wait for the action server to start
        rospy.loginfo("Waiting for action server...")
        # self.client.wait_for_server()
        rospy.loginfo("Action server found!")

    def add_patrol(self, patrol):
        pass

    def update_patrol(self, x):
        self.patrols_data.update(x)
        print("Hola Soy el scheduler patrol", x)
        self.update_patrols_view.emit(self.patrols_data)

    def schedule_all_patrols(self):
        pass

    def sort_key(self, patrol):
        today = 0
        days_until = (0 - today) % 7
        print(int(f'{days_until}{patrol["time"]}'))
        return int(f'{days_until}{patrol["time"]}')

    def sort_patrols(self):
        return sorted(self.patrols, key=self.sort_key)

    def stop(self):
        pass

    def update(self):
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
        print(client.patrols)
        print(client.sort_patrols())
        # result = client.send_goal()
        # rospy.loginfo("Final result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
