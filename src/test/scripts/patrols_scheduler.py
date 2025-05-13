#!/usr/bin/env python
import rospy 
import actionlib
from actionlib_msgs.msg import GoalStatus
# Import your action message, for example: from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot
from datetime import datetime
import time


class ScheduleChecker(QThread):
    progress_updated = pyqtSignal(int)
    task_completed = pyqtSignal(str)

    def __init__(self, task_id=1, patrols=None):
        super().__init__()
        self.task_id = task_id

    def run(self):
        try:
            for i in range(1, 10):
                # Simulate work
                time.sleep(1)

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
    update_patrols_view = pyqtSignal(dict)
    add_patrol_view = pyqtSignal(dict)

    def __init__(self, id=5, date=None):
        super().__init__()
        self.id = id
        self.date = date
        self.patrols = []
        self.scheduler = None

        # self.patrols_data = {
        #     "0": {
        #         "days": {
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        #             "Mar": {"day": "Mar", "time": 20202, "finished": False},
        #             "Mie": {"day": "Mar", "time": 20207, "finished": False},
        #         },
        #         "time": 200,
        #         "state": "encurso",
        #     },
        #     "1": {
        #         "days": {
        #             "Lun": {"day": "Lun", "time": 20200, "finished": False},
        #             "Mar": {"day": "Mar", "time": 20209, "finished": False},
        #             "Mie": {"day": "Mar", "time": 20203, "finished": False},
        #         },
        #         "time": 200,
        #         "state": "encurso",
        #     },
        # }
        self.patrols_data = {}
        self.patrols_for_scheduling = self.patrols.copy()
        self.scheduled_patrols = []
        self.isScheduling = True
        self.isPatrolArrayModified = False

        # Wait for the action server to start
        rospy.loginfo("Waiting for action server...")
        # self.client.wait_for_server()
        rospy.loginfo("Action server found!")

    def start_patrols_scheduling(self):
        if self.scheduler and self.scheduler.isRunning():
            return

        self.scheduler = ScheduleChecker()
        # self.worker_thread = WorkerThread(task_id=1)
        self.scheduler.progress_updated.connect(self.update_progress)
        self.scheduler.task_completed.connect(self.task_finished)
        self.scheduler.start()

    def update_progress(self, k):
        print(f'updated {k}')

    def cancel_task(self):
        if self.scheduler and self.scheduler.isRunning():
            self.scheduler.requestInterruption()

    def task_finished(self, message):
        print(message)
        # self.start_button.setEnabled(True)
        # self.cancel_button.setEnabled(False)
        if self.scheduler:
            self.scheduler.quit()
            self.scheduler.wait()
            self.scheduler = None

    def update_patrol(self, x):
        self.patrols_data.update(x)
        print("Hola Soy el scheduler patrol", x)
        self.update_patrols_view.emit(self.patrols_data)

    def delete_patrols(self, xs):
        for x in xs:
            self.patrols_data.pop(x)
        self.update_patrols_view.emit(self.patrols_data)

    def sort_key(self, patrol):
        days_shortname = [ "Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        day = days_shortname.index(patrol['day'])
        now = datetime.now()
        today = now.weekday()
        days_until = (day - today) % 7
        print(f'{days_until}{patrol["time"]}  day: {day} today: {today} day: {patrol["day"]}')
        return int(f'{days_until}{patrol["time"]}')

    def start_patrols(self):
        self.patrols = []

        for id, patrol in self.patrols_data.items():
            for patrol_day in patrol["days"].values():
                self.patrols.append(patrol_day)
        print(self.patrols)
        self.patrols.sort(key=self.sort_key)
        print(self.patrols)
        self.start_patrols_scheduling()



if __name__ == "__main__":
    try:
        client = PatrolsEscheduler()
        print(client.patrols)
        print(client.sort_patrols())
        # result = client.send_goal()
        # rospy.loginfo("Final result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
