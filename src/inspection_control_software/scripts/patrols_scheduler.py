#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
# Import your action message, for example: from test.msg import PatrolAction, PatrolGoal, PatrolResult, PatrolFeedback

from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot
from datetime import datetime
import time

from points_scheduler import PointsScheduler
from database_manager import DataBase
import queue
from enum import Enum


from pyqttoast import Toast, ToastPreset


from utils.patrol import PatrolEndState


class ScheduleChecker(QThread):
    progress_updated = pyqtSignal(int)
    task_completed = pyqtSignal(str)
    patrol_state = pyqtSignal(str)
    weekday_changed = pyqtSignal(int)
    patrol_delayed = pyqtSignal(str)

    def __init__(
        self,
        task_id=1,
        patrols=[],
        currentpatrol={},
        points_scheduler=None,
        forced_exec=False,
        single_patrolid=None,
        parent=None
    ):
        super().__init__()
        self.task_id = task_id
        self.patrols = patrols
        self.index = currentpatrol
        self.points_scheduler = points_scheduler
        self.isDispatching = False
        self.isForcingPatrolExec = forced_exec
        self.single_patrolid = single_patrolid
        self.isForcedCancel = False
        self.isSinglePatrolFinished = False
        self.isBusy = False
        self.wainting_queue = []
        self.dispatched_patrols = []
        self.movePatrolsIndex = False
        self.sleep_time = 5
        self.last_weekday = self.current_weekday = datetime.now().weekday()
        self.ignoreTimeDiff = False
        # self.forced_exec = forced_exec

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
            if self.isForcingPatrolExec:
                self.points_scheduler.setDoneTask(
                    done_task=self.handlePatrolsForcedExec
                )
                self.patrol_state.emit(str(self.single_patrolid))
                self.points_scheduler.dispatch(self.single_patrolid)

                while not self.isSinglePatrolFinished:
                    print("EXEC FORCED PATROL...")
                    if self.isInterruptionRequested():
                        self.points_scheduler.cancel_points_scheduling()
                        self.task_completed.emit("TaskCancelled")
                        return

                    self.sleep(self.sleep_time)

                self.task_completed.emit("TaskSuccessfully")
                return
        except Exception as e:
            # raise e
            print("Algo sali mal en FORCED PATROL")
            self.task_completed.emit(f"TaskFailedForced: {str(e)}")
            return

        try:
            # while True:
            # print('HOLA HAROLD')
            while self.index["currentpatrol_index"] < len(self.patrols):
                # Simulate work
                if self.isInterruptionRequested():
                    self.points_scheduler.cancel_points_scheduling()
                    self.task_completed.emit("TaskCancelled")
                    return

                if not self.last_weekday == self.current_weekday:
                    self.weekday_changed.emit(self.current_weekday)

                self.current_weekday = datetime.now().weekday()

                # self.sleep(self.sleep_time)
                scheduled_patrol = self.patrols[self.index["currentpatrol_index"]]
                now = datetime.now()
                day, hour, minute = self.getDatetime(scheduled_patrol)
                print(
                    "TASK WORKER index day hour minute",
                    self.index["currentpatrol_index"],
                    day - now.weekday(),
                    (hour - now.hour),
                    (minute - now.minute),
                )

                if day == now.weekday() and hour == now.hour and minute == now.minute:
                    # continue

                    # print('PATRULLAJE', scheduled_patrol)

                    if not self.isDispatching:
                        print("worker thread points schedule: ", scheduled_patrol)
                        if not scheduled_patrol["finished"]:
                            self.patrol_state.emit(scheduled_patrol.get("patrolid"))
                            self.points_scheduler.setDoneTask(
                                done_task=self.free_points_dispatcher
                            )
                            self.points_scheduler.dispatch(scheduled_patrol["patrolid"])
                            self.isDispatching = True
                            scheduled_patrol["finished"] = True

                    self.ignoreTimeDiff = True

                elif now.weekday() == day and not self.ignoreTimeDiff:
                    if hour - now.hour < 0:
                        self.movePatrolsIndex = True
                        self.patrol_delayed.emit(scheduled_patrol["patrolid"])
                    elif  hour - now.hour == 0:
                        if minute - now.minute < 0:
                            self.movePatrolsIndex = True
                            self.patrol_delayed.emit(scheduled_patrol["patrolid"])

                if self.movePatrolsIndex:
                    self.index["currentpatrol_index"] = (
                        self.index["currentpatrol_index"] + 1
                    )
                    self.movePatrolsIndex = False
                    self.ignoreTimeDiff = False
                    pass

                self.sleep(self.sleep_time)

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
        self.isDispatching = False
        self.points_scheduler.restart()
        print("FREEE POINT DISPACHER!!!!!!")

    def handlePatrolsForcedExec(self):
        self.isSinglePatrolFinished = True
        self.points_scheduler.restart()
        pass

    def forced_cancel(self, x):
        self.isForcedCancel = True


class PatrolsEscheduler(QObject):
    compelted_patrols = []
    looped_patrols = []
    update_patrols_view = pyqtSignal(dict)
    patrol_finished = pyqtSignal(str)
    patrol_progress = pyqtSignal(int)
    patrols_scheduling_state = pyqtSignal(str)
    set_running_patrol = pyqtSignal(str)
    terminate_all_patrols = pyqtSignal(str)
    single_patrol_active = pyqtSignal(str)
    set_stored_database_points = pyqtSignal(dict)
    weekday_changed = pyqtSignal(int)
    patrol_delayed = pyqtSignal(str)
    # add_patrol_view = pyqtSignal(dict)

    def __init__(self, id=5, date=None, parent=None):
        super().__init__()
        self.id = id
        self.date = date
        self.patrols = []
        self.scheduler = None
        self.parent = parent
        self.forced_scheduler = None
        self.single_patrolid = None
        self.patrolIsRunning = False
        self.indexKeeper = {"currentpatrol_index": 0}  # to pass as a pointer
        self._points_to_visit = []
        self.points_scheduler = PointsScheduler()
        self.database = None
        self.actions_queue = []

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
        self.forceExec = False

        self.send_database_action(action="get_user_patrols")

    # def patrols_schedule_generator(self):
    # for patrol in self.
    def send_database_action(self, action: str, data: dict = {}):
        if self.database and self.database.isRunning():
            self.actions_queue.append((action, data))
            return

        self.database = DataBase(action=action, data=data)
        self.database.action_completed.connect(self.database_action_finished)
        self.database.start()

    def database_action_finished(self, state, data):
        # self.patrols_data.update(data)
        if state == "SuccessGetAllUserPatrols":
            self.patrols_data.update(data)
            self.update_patrols_view.emit(self.patrols_data)

        if state == "SuccessSavePatrol":
            pass

        if state =='SuccessSavePoints':
            toast = Toast(self.parent)
            toast.setDuration(5000)  # Hide after 5 seconds
            toast.setTitle("Puntos Guardados exitosamente")
            toast.setText("Puntos Guardados exitosamente")
            toast.applyPreset(ToastPreset.SUCCESS)  # Apply style preset
            Toast.setPositionRelativeToWidget(self.parent)
            toast.show()

        if state == "SuccessGetPoinst":
            print("SCHEDULER", data)
            self.set_stored_database_points.emit(data)

        self.database.quit()
        self.database.wait()
        self.database = None

        if len(self.actions_queue) > 0:
            action, data1 = self.actions_queue.pop(0)
            self.database = DataBase(action=action, data=data1)
            self.database.action_completed.connect(self.database_action_finished)
            self.database.start()

    def start_patrols_scheduling(self, patrol=None):
        if self.scheduler and self.scheduler.isRunning():
            return

        self.scheduler = ScheduleChecker(
            currentpatrol=self.indexKeeper,
            patrols=self.patrols,
            points_scheduler=self.points_scheduler,
            parent=self.parent
        )
        self.scheduler.progress_updated.connect(self.update_progress)
        self.scheduler.task_completed.connect(self.task_finished)
        self.scheduler.patrol_state.connect(self.update_patrol_info)
        self.scheduler.weekday_changed.connect(self.handleWeekDayChanged)
        self.scheduler.patrol_delayed.connect(self.handlePatrolDelayed)

        # self.terminate_all_patrols.connect(self.scheduler.forced_cancel)
        self.scheduler.start(QThread.IdlePriority)
        self.patrolIsRunning = True

    def start_single_patrol_scheduling(self, patrolid):
        self.single_patrolid = patrolid
        if not self.cancel_task():
            self.exec_single_patrol()
        pass

    def exec_single_patrol(self):
        self.scheduler = ScheduleChecker(
            currentpatrol=self.indexKeeper,
            single_patrolid=self.single_patrolid,
            forced_exec=True,
            # patrols=[self.patrols_data.get(patrolid)],
            points_scheduler=self.points_scheduler,
            parent=self.parent
        )
        print("FORCED EXECUTION patrol id:", self.single_patrolid)
        # self.scheduler.start(QThread.IdlePriority)
        self.points_scheduler.setGoals()
        self.scheduler.progress_updated.connect(self.update_progress)
        self.scheduler.task_completed.connect(self.task_finished)
        self.scheduler.patrol_state.connect(self.update_patrol_info)

        self.scheduler.start(QThread.IdlePriority)

        self.single_patrol_active.emit(str(self.single_patrolid))
        self.single_patrolid = None
        self.patrolIsRunning = True

    def update_progress(self, k):
        # a = self.indexKeeper["currentpatrol_index"]
        # self.patrol_progress.emit(a)
        # print(f"updated {a}")
        pass

    def cancel_task(self) -> bool:
        self.indexKeeper["currentpatrol_index"] = 0
        # if self.scheduler.isForcingPatrolExec:
        #     self.points_scheduler.cancel_points_scheduling()
        #     return True

        if self.scheduler and self.scheduler.isRunning():
            self.scheduler.requestInterruption()
            print("scheduler request cancel")
            self.patrolIsRunning = False
            return True  # patrol was cancel?
        return False  # patrol was cancel?

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
        print("patrol killed", self.scheduler)
        self.patrol_finished.emit(message)

        if self.single_patrolid:
            print("single patrol func executing")
            self.exec_single_patrol()

    def update_patrol(self, x):
        [id] = list(x.keys())
        print("update PATROL ID verifcation", self.patrols_data.get(id))

        if self.patrols_data.get(id):
            self.send_database_action(action="update_patrol", data=x)
        else:
            self.send_database_action(action="save_patrol", data=x)

        self.patrols_data.update(x)
        print("Hola Soy el scheduler patrol", x)
        self.update_patrols_view.emit(self.patrols_data)

    def delete_patrols(self, patrolsToDelete):
        for patrol in patrolsToDelete:
            self.patrols_data.pop(patrol)
        self.send_database_action(
            action="delete_user_patrols", data={"ids": patrolsToDelete}
        )
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
                self.patrols_data[id]['delayed'] = None 
                self.patrols_data[id]['state'] = None 

        self.update_patrols_view.emit(self.patrols_data)

        print(self.patrols)
        self.patrols.sort(key=self.sort_key)
        now = datetime.now()
        days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        delayed = [
            patrol for patrol in self.patrols if self.filter_delayed_patrols(patrol)
        ]
        ontime = [
            patrol for patrol in self.patrols if not self.filter_delayed_patrols(patrol)
        ]

        [
            patrol.update(
                {"date_day": (days_shortname.index(patrol.get("day")) - now.weekday())%7}
            )
            for patrol in ontime
        ]

        self.patrols = []
        self.patrols.extend(ontime)
        self.patrols.extend(delayed)
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        # for patrol in self.patrols:
        # patrol['date'] =

        print("ORDER PATROL", len(ontime), len(delayed), self.patrols)

        self.points_scheduler.setGoals()
        self.start_patrols_scheduling()
        self.patrols_scheduling_state.emit("start")
        self.weekday_changed.emit(datetime.now().weekday())

    def filter_delayed_patrols(self, patrol):
        day, hour, minute = self.getDatetime(patrol)
        now = datetime.now()
        if now.weekday() == day:
            if hour - now.hour < 0:
                # temp.append(self.patrols.pop(index))
                # patrol_index_todelete.append(index)
                return True
            elif hour - now.hour == 0:
                if minute - now.minute < 0:
                    # patrol_index_todelete.append(index)
                    return True
                    # temp.append(self.patrols.pop(index))
        return False

    def update_patrol_info(self, id):
        print("UPDATE PATROL INFO")
        self.set_running_patrol.emit(id)

    def handlePatrolDelayed(self, patrolid):
        self.patrol_delayed.emit(patrolid)

    def setPointsToVisit(self, points):
        self._points_to_visit = points
        self.points_scheduler.update_points(points)

    def handleSavePointsInDatabase(self, points_to_save):
        self.send_database_action(action="save_points", data=points_to_save)

    def send_points_data(self, m):
        self.send_database_action(action="get_points", data={"map_file": m})
        # self.get_stored_database_points.emit()

    def handleWeekDayChanged(self, current_weekday):
        print("CURRENT WEEKDAY", current_weekday)
        self.weekday_changed.emit(current_weekday)

    def getDatetime(self, patrol):
        days_shortname = ["Lun", "Mar", "Mie", "Jue", "Vie", "Sab", "Dom"]
        day = days_shortname.index(patrol["day"])
        patrol_time = list(patrol["time"])
        hour = "".join(patrol_time[:2])
        minute = "".join(patrol_time[2:])
        #             "Lun": {"day": "Lun", "time": 20205, "finished": False},
        return day, int(hour), int(minute)


if __name__ == "__main__":
    try:
        client = PatrolsEscheduler()
        print(client.patrols)
        print(client.sort_patrols())
        # result = client.send_goal()
        # rospy.loginfo("Final result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
