#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
# Import your action message, for example: from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot
from datetime import datetime
import time

from points_scheduler import PointsScheduler


class ScheduleChecker(QThread):
    progress_updated = pyqtSignal(int)
    task_completed = pyqtSignal(str)
    patrol_state = pyqtSignal(str)

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
        self.dispatched_patrols = []
        self.movePatrolsIndex = False

        self.state = "red"
        self.next_state = None

    # def transitions(self):

    def getDatetime(self, patrol):
        days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        day = days_shortname.index(patrol["day"])
        patrol_time = list(patrol["time"])
        hour = "".join(patrol_time[:2])
        minute = "".join(patrol_time[2:])
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        return day, int(hour), int(minute)

    def run(self):
        try:
            # while True:
            # print('HOLA HAROLD')
            while self.index["currentpatrol_index"] < len(self.patrols):
                self.sleep(15)
                # Simulate work
                if self.isInterruptionRequested():
                    self.task_completed.emit(f"TaskCancelled")
                    return

                scheduled_patrol = self.patrols[self.index["currentpatrol_index"]]
                now = datetime.now()
                day, hour, minute = self.getDatetime(scheduled_patrol)
                print('TASK WORKER', self.index["currentpatrol_index"] , (hour - now.hour), (minute - now.minute))

                if (hour - now.hour) < 0:
                    if -6 <  day - now.weekday():
                        self.index["currentpatrol_index"] = (
                            self.index["currentpatrol_index"] + 1
                        )
                        print('PATROL DELAYED',self.index["currentpatrol_index"],  )
                        continue
                elif (hour - now.hour) == 0:
                    if (minute - now.minute) < 0:
                        self.index["currentpatrol_index"] = (
                            self.index["currentpatrol_index"] + 1
                        )
                        print('PATROL DELAYED',self.index["currentpatrol_index"],  )
                        continue


                if not (
                    day == now.weekday() and hour == now.hour and minute == now.minute
                ):
                    continue

                # print('PATRULLAJE', scheduled_patrol)

                if not self.isDispatching:
                    print("worker thread points schedule: ", scheduled_patrol)
                    self.patrol_state.emit(str(scheduled_patrol.get("patrolid")))
                    if not scheduled_patrol["finished"]:
                        self.points_scheduler.setDoneTask(
                            done_task=self.free_points_dispatcher
                        )
                        self.points_scheduler.dispatch(scheduled_patrol["patrolid"])
                        self.isDispatching = True
                        scheduled_patrol["finished"] = True

                if self.movePatrolsIndex:
                    self.index["currentpatrol_index"] = (
                        self.index["currentpatrol_index"] + 1
                    )
                    self.movePatrolsIndex = False

                # Update progress
                # self.progress_updated.emit(999)

                # Check if thread should stop
                # if self.isInterruptionRequested():
                #     self.task_completed.emit(f"Task {self.task_id} cancelled")
                #     return

            # Task completed
            print("TERMINO")
            self.task_completed.emit("TaskSuccessfully")
        except Exception as e:
            pass
            self.task_completed.emit(f"TaskFailed: {str(e)}")
            toast = Toast(self.parent)
            toast.setDuration(4000)  # Hide after 5 seconds
            toast.setTitle("ERROR: En la ejecion de las patrullas")
            toast.setText("Inicie nuevamente las patrullas")
            toast.applyPreset(ToastPreset.ERROR)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

    def free_points_dispatcher(self):
        self.movePatrolsIndex = True

        # if self.index["currentpatrol_index"] < len(self.patrols):
        self.isDispatching = False
        self.points_scheduler.restart()

    def handlePatrolsForcedExec(self, patrol):
        self.isForcingPatrolExec = True
        pass


class PatrolsEscheduler(QObject):
    compelted_patrols = []
    looped_patrols = []
    update_patrols_view = pyqtSignal(dict)
    patrol_finished = pyqtSignal(str)
    patrol_progress = pyqtSignal(int)
    patrols_scheduling_state = pyqtSignal(str)
    patrol_state = pyqtSignal(str)
    # add_patrol_view = pyqtSignal(dict)

    def __init__(self, id=5, date=None):
        super().__init__()
        self.id = id
        self.date = date
        self.patrols = []
        self.scheduler = None
        self.patrolIsRunning = False
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
        self.scheduler.patrol_state.connect(self.update_patrol_info)
        self.scheduler.start(QThread.IdlePriority)
        self.patrolIsRunning = True

    def update_progress(self, k):
        a = self.indexKeeper["currentpatrol_index"]
        self.patrol_progress.emit(a)
        print(f"updated {a}")

    def cancel_task(self):
        self.indexKeeper["currentpatrol_index"] = 0
        if self.scheduler and self.scheduler.isRunning():
            self.scheduler.requestInterruption()
            self.scheduler = None
            return True
        return False

    def task_finished(self, message):
        self.indexKeeper["currentpatrol_index"] = 0
        self.patrolIsRunning = False
        print(message)
        # self.start_button.setEnabled(True)
        # self.cancel_button.setEnabled(False)
        if self.scheduler:
            self.scheduler.quit()
            self.scheduler.wait()
            self.scheduler = None
        self.patrol_finished.emit(message)

    def update_patrol(self, x):
        self.patrols_data.update(x)
        print("Hola Soy el scheduler patrol", x)
        self.update_patrols_view.emit(self.patrols_data)

    def delete_patrols(self, patrolsToDelete):
        for patrol in patrolsToDelete:
            self.patrols_data.pop(patrol)
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
        self.patrols_scheduling_state.emit("start")

    def update_patrol_info(self, id):
        self.patrol_state.emit(id)

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
