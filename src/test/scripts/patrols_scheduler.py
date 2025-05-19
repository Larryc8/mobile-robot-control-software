#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
# Import your action message, for example: from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot
from datetime import datetime
import time

from points_scheduler import PointsScheduler


class ScheduleStateMachine:
    def __init__(self):
        self.state = "red"
        self.transitions = {
            "red": {"next": "green", "action": self.turn_green},
            "green": {"next": "yellow", "action": self.turn_yellow},
            "yellow": {"next": "red", "action": self.turn_red}
        }
    
    def turn_green(self):
        print("Turning light green - Go!")
    
    def turn_yellow(self):
        print("Turning light yellow - Caution!")
    
    def turn_red(self):
        print("Turning light red - Stop!")
    
    def change(self):
        transition = self.transitions[self.state]
        transition["action"]()
        self.state = transition["next"]
        print(f"Light is now {self.state}")

class ScheduleChecker(QThread):
    progress_updated = pyqtSignal(int)
    task_completed = pyqtSignal(str)

    def __init__(self, task_id=1, patrols=[], currentpatrol={}, points_scheduler=None):
        super().__init__()
        self.task_id = task_id
        self.patrols = patrols
        self.index = currentpatrol
        self.points_scheduler = points_scheduler
        self.isDispatching = False
        self.isForcingPatrolExec = False
        self.isBusy = False
        self.wainting_queue = []

        self.state = "red"
        self.next_state = None

    # def transitions(self):
       

    def change(self):
        transition = self.transitions[self.state]
        transition["action"]()
        self.state = transition["next"]
        print(f"Light is now {self.state}")

    def getDatetime(self, patrol):
        days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        day = days_shortname.index(patrol["day"])
        time = list(patrol['time'])
        hour = ''.join(time[:2])
        minute = ''.join(time[2:])
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        return day, int(hour), int(minute)


    def run(self):
        try:
            while self.index["currentpatrol_index"] < len(self.patrols):
                # Simulate work

                scheduled_patrol = self.patrols[self.index["currentpatrol_index"]]
                now = datetime.now()
                day, hour, minute = self.getDatetime(scheduled_patrol)
                # print('TASK WORKER', now.weekday(), now.hour, now.minute)

                if not (day == now.weekday() and hour == now.hour and minute == now.minute):
                    pass
                else:
                    if not self.isDispatching:
                        print("worker thread points schedule: ", scheduled_patrol)
                        self.points_scheduler.setDoneTask(
                            done_task=self.free_points_dispatcher
                        )
                        self.points_scheduler.dispatch()
                        self.isDispatching = True

                # self.index["currentpatrol_index"] = (
                #     self.index["currentpatrol_index"] + 1
                # )

                # Update progress
                # self.progress_updated.emit(999)

                # Check if thread should stop
                if self.isInterruptionRequested():
                    self.task_completed.emit(f"Task {self.task_id} cancelled")
                    return

            # Task completed
            self.task_completed.emit(f"Task {self.task_id} finished successfully")
        except Exception as e:
            self.task_completed.emit(f"Task {self.task_id} failed: {str(e)}")

    def free_points_dispatcher(self):
        self.isDispatching = False
        self.points_scheduler.restart()
        self.index["currentpatrol_index"] = self.index["currentpatrol_index"] + 1
        pass

    def handlePatrolsForcedExec(self, patrol):
        self.isForcingPatrolExec = True
        pass


class PatrolsEscheduler(QObject):
    compelted_patrols = []
    looped_patrols = []
    update_patrols_view = pyqtSignal(dict)
    patrol_finished = pyqtSignal(bool)
    patrol_progress = pyqtSignal(int)
    patrols_scheduling_state = pyqtSignal(str)
    # add_patrol_view = pyqtSignal(dict)

    def __init__(self, id=5, date=None):
        super().__init__()
        self.id = id
        self.date = date
        self.patrols = []
        self.scheduler = None
        self.indexKeeper = {"currentpatrol_index": 0}  # to pass as a pointer
        self._points_to_visit = []
        self.points_scheduler = PointsScheduler()

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

    # def patrols_schedule_generator(self):
    # for patrol in self.

    def start_patrols_scheduling(self, patrol=None):
        if self.scheduler and self.scheduler.isRunning():
            return

        self.scheduler = ScheduleChecker(
            currentpatrol=self.indexKeeper,
            patrols=self.patrols,
            points_scheduler=self.points_scheduler,
        )
        self.scheduler.progress_updated.connect(self.update_progress)
        self.scheduler.task_completed.connect(self.task_finished)
        self.scheduler.start()

    def update_progress(self, k):
        a = self.indexKeeper["currentpatrol_index"]
        self.patrol_progress.emit(a)
        print(f"updated {a}")

    def cancel_task(self):
        self.indexKeeper["currentpatrol_index"] = 0
        if self.scheduler and self.scheduler.isRunning():
            self.scheduler.requestInterruption()

    def task_finished(self, message):
        self.indexKeeper["currentpatrol_index"] = 0
        print(message)
        # self.start_button.setEnabled(True)
        # self.cancel_button.setEnabled(False)
        if self.scheduler:
            self.scheduler.quit()
            self.scheduler.wait()
            self.scheduler = None
        self.patrol_finished.emit(True)

    def update_patrol(self, x):
        self.patrols_data.update(x)
        print("Hola Soy el scheduler patrol", x)
        self.update_patrols_view.emit(self.patrols_data)

    def delete_patrols(self, xs):
        for x in xs:
            self.patrols_data.pop(x)
        self.update_patrols_view.emit(self.patrols_data)

    def get_current_patrols(self):
        self.update_patrols_view.emit(self.patrols_data)

    def sort_key(self, patrol):
        days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        day = days_shortname.index(patrol["day"])
        now = datetime.now()
        today = now.weekday()
        days_until = (day - today) % 7
        print(
            f'{days_until}{patrol["time"]}  day: {day} today: {today} day: {patrol["day"]}'
        )
        return int(f'{days_until}{patrol["time"]}')

    def start_patrols(self):
        self.patrols = []

        for id, patrol in self.patrols_data.items():
            for patrol_day in patrol["days"].values():
                self.patrols.append(patrol_day)
        print(self.patrols)
        self.patrols.sort(key=self.sort_key)
        print(self.patrols)
        self.points_scheduler.setGoals()
        self.start_patrols_scheduling()
        self.patrols_scheduling_state.emit('start')


    def setPointsToVisit(self, points):
        self._points_to_visit = points
        self.points_scheduler.update_points(points)


if __name__ == "__main__":
    try:
        client = PatrolsEscheduler()
        print(client.patrols)
        print(client.sort_patrols())
        # result = client.send_goal()
        # rospy.loginfo("Final result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
