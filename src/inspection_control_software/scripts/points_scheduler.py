#!/usr/bin/env python
import logging
import math
import os
import queue
import sys
import time
from ast import Dict, List
from math import degrees, radians
from typing import Any, Callable, NamedTuple

import actionlib
import cv2 as cv

# if __name__ != "__main__":
import ImageSimilarity.image_similarity2 as imgsim
import numpy as np
import robot_actions_logger
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from config_model import UserConfigFileManager
from cv_bridge import CvBridge
from database_manager import AlertStatus, DataBase
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseFeedback,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import Odometry
from pose_controller import DifferentialDriveController
from PyQt5.QtCore import (  # , pyqtSlot
    QObject,
    QRect,
    QSize,
    QThread,
    QTimer,
    pyqtSignal,
)
from robot_actions_logger import add_color
from robot_navigation_checker import RobotNavigationChecker, StuckDetector
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import BatteryState, Image
from utils.patrol import PatrolEndState
from utils.sorting import sort_nearest_neighbor


class Pose(NamedTuple):
    x: float = 0
    y: float = 0


class PointsScheduler(QObject):
    points_state = pyqtSignal(str, str, int)
    ## current pointid, next point id, numbres of points tha left, total points
    patrol_progress = pyqtSignal(str, int, int, PatrolEndState)
    alert_generated = pyqtSignal(str, AlertStatus)
    check_done = pyqtSignal(int)

    def __init__(self, points=[], done_task=None, feedback_task=None) -> None:
        super().__init__()
        # rospy.init_node("action_client_move_base")
        # self._goals = [
        #     {"x_meters": 1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": 1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": -1.7, "y_meters": 1.1, "yaw_degrees": 0, "checked": False},
        #     {"x_meters": -1.7, "y_meters": -1.1, "yaw_degrees": 0, "checked": False},
        # ]
        self.currrent_pose = (None, None, None)
        self.currrent_aruco_pose = (None, None, None)
        self.scan_angles = [180, 100, 90, 60, 30, 0]
        self.current_yaw = None
        self.target_yaw = None
        self.current_point_calibration = None
        self.database = None
        self.on_calibration = False
        self.current_patrolid = None
        self._goals = {}
        self.goals = {}
        # self.goals = self._goals.copy()
        self.done_task = done_task
        self.feedback_task = feedback_task
        self.cancelled = False
        self.emit_callback = None
        self.client = None
        self.points_left = 999
        self.map = None
        self.current_map_filename: str = ""
        self.current_pointid = None
        self.init_pose = Pose(0, 0)
        self.battery_state = 100

        self.current_position_x = 0
        self.current_position_y = 0

        self.bridge = CvBridge()
        self.camera_image_filepath = "/pico-sdk/mobile-robot-control-software/src/inspection_control_software/scripts/reference_images/camera.jpg"

        self.track = [0]
        self.user_config = UserConfigFileManager()
        self.navigation_checker = RobotNavigationChecker(self.track)
        self.stuck_detector = StuckDetector()

        self.battery_state_sub = rospy.Subscriber(
            "/battery_state", BatteryState, self.update_battery
        )
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.pose_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback
        )
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.simple_goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=2
        )

        self.navigation_checker.alert_generated.connect(self.handleAlertGeneration)

        # actionlib.GoalStatus.SUCCEEDED
        #
        #

    def update_battery(self, msg):
        self.battery_state = msg.percentage * 100

    def setup(self, on_calibration: bool = False) -> int:
        """
        this function sets the initials conditions for the patrols scheduler
        sort the point to make the robot follow the shrortest path
        """
        points: dict = self._goals.copy()
        new_goals: dict
        self.on_calibration = on_calibration
        self.cancelled = False

        if on_calibration:
            new_goals = {
                id: points.get(id)
                for id in points.keys()
                if points.get(id).get("image")
            }
        else:
            new_goals = points

        print("NEW POIINT", new_goals)

        new_goals, new_goals_original = sort_nearest_neighbor(
            new_goals,
            current_position=(
                "1",
                {
                    "x_meters": self.current_position_x,
                    "y_meters": self.current_position_y,
                },
            ),
        )

        print("NEW POIINT", new_goals)
        self._goals = new_goals_original.copy()
        self.goals = new_goals.copy()

        return len(self._goals)

    def handleAlertGeneration(self, message, status):
        print(f"POINT SCHEDULER {message}")
        self.alert_generated.emit(message, status)

    def setGoals(self):
        self.goals = self._goals.copy()
        self.goals_count = len(self.goals)
        self.points_left = len(self.goals)

        return self.goals_count

    def configGoal(self, x, y, yaw):
        quaternion = Rotation.from_euler("z", yaw).as_quat()
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

    def amcl_pose_callback(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y

    def aruco_odom_callback(self, odom_msg):
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        # self.current_pose.theta = yaw
        self.currrent_aruco_pose = (
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            yaw,
        )

    def setHomePoint(self):
        pass

    def setMap(self, map):
        self.map = map

    def cancel_points_scheduling(self):
        if self.client:
            if self.client.get_state() == GoalStatus.ACTIVE:
                self.client.cancel_goal()
                # self.done_task()
                # self.restart()
        self.cancelled = True
        print("points canceled from points sche", self.cancelled)

    def dispatch(self, patrolid=None):
        print(f"{__name__} dispatched!! len(goals): {len(self.goals)}")
        self.current_patrolid = patrolid
        self.patrol_progress.emit(
            self.current_patrolid,
            self.points_left,
            self.goals_count,
            PatrolEndState.ACTIVE,
        )

        if len(self.goals) > 0:
            # self.patrol_progress.emit(self.current_patrolid, len(self.goals), len(self._goals))
            self.current_pointid, pose = self.goals.popitem()
            print(
                "points points_scheduler points",
                len(self.goals),
                pose.get("x_meters"),
                pose.get("y_meters"),
            )  # ids_list = list(self.goals.keys()) if len(ids_list) < 2: self.points_state.emit(self.id, None, len(self.goals), len(self._goals)) else:
            #     self.points_state.emit(id, ids_list[-1], len(self.goals), len(self._goals))
            # x_meters, y_meters, yaw_degrees, check = pose.values()
            x_meters, y_meters, check, yaw, image = (
                pose.get("x_meters"),
                pose.get("y_meters"),
                pose.get("checked"),
                pose.get("yaw"),
                pose.get("image"),
            )
            self.currrent_pose = (x_meters, y_meters, yaw)
            self.current_reference_image_path = image
            self.current_map_filename = pose.get("mapfile")
            self.aruco_pose = pose.get("aruco_pose_vector")

            goal = self.configGoal(x_meters, y_meters, yaw)
            # self.navigation_checker = RobotNavigationChecker()
            self.navigation_checker.set_current_goal(goal, self.points_left)
            self.navigation_checker.start_checker(
                patrol_id=self.current_patrolid,
                checkpoint_id=self.current_pointid,
                map=self.map,
            )

            self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            self.client.wait_for_server(rospy.Duration(5))
            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def restart(self):
        self.goals = self._goals.copy()
        self.points_left = len(self.goals)
        # self.points_state.emit(None, None, 0)

    def setDoneTask(self, done_task):
        self.done_task = done_task

    def done_cb(self, state, result):
        ids_list = list(self.goals.keys())
        if len(ids_list) == 0:
            self.points_state.emit(self.current_pointid, None, state)
        else:
            # if state in [0,1,3]:
            self.points_state.emit(
                self.current_pointid,
                ids_list[-1],
                state,
            )

        robot_actions_logger.logger.log(
            f"Punto revisado con exito {self.goals_count - len(self.goals)} de {self.goals_count}"
        )
        self.check_done.emit(1)

        # self.patrol_progress.emit(self.current_patrolid, len(self.goals), len(self._goals))
        rospy.loginfo("Finished in state %s %s", str(state), str(len(self.goals)))
        if state in [1, 0, 3]:
            if not len(self.goals) == self.goals_count:
                self.points_left = self.points_left - 1
                self.subroutines_wrapper()

        if len(self.goals) == 0:
            self.patrol_progress.emit(
                self.current_patrolid,
                self.points_left,
                self.goals_count,
                PatrolEndState.FINISHED,
            )
            if self.done_task:
                self.done_task()
            print("TODAS LOS PUNTOS HAN SIDO RECORRIDOS")
            return

        if self.cancelled:
            print("POINTS CANCELLED WITH state ", state)
            return

        self.dispatch(str(self.current_patrolid))

    def active_cb(self):
        rospy.loginfo("Goal just went active")
        self.track[0] = self.points_left
        # self.navigation_checker.listen()
        self.patrol_progress.emit(
            self.current_patrolid,
            self.points_left,
            self.goals_count,
            PatrolEndState.ACTIVE,
        )
        self.points_state.emit(self.current_pointid, self.current_pointid, 0)

    def feedback_cb(self, feedback):
        if self.feedback_task:
            self.feedback_task(feedback)
        if self.client.get_state() == GoalStatus.ACTIVE:
            # self.cancel_goal()
            pass
        # if self.emit_callback:
        # self.emit_callback(str(self.current_patrolid), len(self.goals), len(self._goals))
        # self.patrol_progress.emit(str(self.current_patrolid), len(self.goals), len(self._goals))
        # rospy.loginfo(f"Got Feedback: {1}")

    # def patrol_progress(self,  callback):
    # self.emit_callback = callback#callback(str(self.current_patrolid), len(self.goals), len(self._goals))

    def update_points(self, points: list):
        self._goals = points.copy()
        print("from points_scheduler POINTS UPDATE")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = self.cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def edit_image(self, path: str):
        image = cv.imread(path)
        file_name = path.split("/")[-1]
        file_path = "/".join(path.split("/")[:-1])
        name = file_name.split(".")[0]
        ext = ".".join(file_name.split(".")[0:])

        rect_size = QSize(100, 90)
        height, width, _ = image.shape  # Width and height of crop
        x, y = (width, height)  # Starting coordinates (top-left corner)
        print(f"{__name__} Tamano de la IMAGEN {image.shape}")

        cropped_image = (
            image  # image[y : y + rect_size.height(), x : x + rect_size.width()]
        )
        print(f"{__name__} Tamano de la IMAGEN {cropped_image.shape}")
        new_path = f"{file_path}/{name}_cropped.{ext}"
        print(f"{__name__}: {new_path}")
        cv.imwrite(new_path, cropped_image)

        return new_path

    def subroutines_wrapper(self):
        x, y, yaw = self.currrent_pose
        yaw_degrees = round(degrees(yaw) % 360)
        delta_yaw = 40
        self.scan_angles = [
            angle % 360
            for angle in range(yaw_degrees - delta_yaw, yaw_degrees + delta_yaw, 10)
        ]
        # self.scan_angles = [yaw_degrees]
        # self.get_image_similarity()
        print("cuurent yaw: ", yaw_degrees)
        print(self.scan_angles)
        robot_actions_logger.logger.log(f"Comenzando scaneo....")

        rate: float = self.get_image_similarity()
        self.current_rate = rate
        robot_actions_logger.logger.log(f"Fin del escaneo.... similaridad: {rate:.3f}")
        # r = self.scan_subroutine(x, y, self.scan_angles.copy())
        # robot_actions_logger.logger.log(
        #     f"Scaneo finalizado resultado de la similaridad en{max(r):.4f}"
        # )

        if self.on_calibration:
            self.save_calibration(
                rate, None, self.current_map_filename, id=self.current_pointid
            )
        else:
            self.current_point_calibration = rate
            self.get_calibration([], id=self.current_pointid)

        # --- NEW SUBROUTINEN
        if False:
            controller = DifferentialDriveController()

            # Example: Set some desired values (you would typically get these from other nodes)
            controller.set_linear_distance_setpoint(self.aruco_pose[:2])  # 0.5 m/s
            yaw = self.aruco_pose[2]
            controller.set_yaw_setpoint(yaw)  # 90 degrees

            control_rate = rospy.Rate(20)  # 50 Hz

            goal_reached = 777
            pos = 888

            while not (pos == 0):
                if self.cancelled:
                    break
                goal_reached, pos = controller.control_loop()
                # print(f"inside the while in  pid controller yaw error: {goal_reached} pose error: {pos}")
                control_rate.sleep()

            pos = 888
            goal_reached = 777
            while not (goal_reached == 0):
                if self.cancelled:
                    break
                goal_reached, pos = controller.control_loop(yaw=True)
                # print(f"inside the while in  pid controller yaw error: {goal_reached} pose error: {pos}")
                control_rate.sleep()

            print("ARUCO GOAL REACHED", goal_reached, pos)

        # rate: float = self.get_image_similarity()
        # robot_actions_logger.logger.log(
        #     f"Fin del escaneo.... after fine-tuning sim: {rate:.3f}"
        # )

    def odom_callback(self, msg):
        """
        Callback function to process the Odometry message.
        """
        # --- Position ---
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        # --- Orientation (Quaternion to Euler) ---
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # Convert yaw from radians to degrees
        yaw_deg = degrees(yaw)
        self.current_yaw = yaw_deg % 360

        # --- Velocities ---
        linear_velocity_x = msg.twist.twist.linear.x
        angular_velocity_z = msg.twist.twist.angular.z  # Yaw rate

        distance: float = math.sqrt(
            (position_x - self.init_pose.x) ** 2 + (position_y - self.init_pose.y) ** 2
        )
        if distance > 0.20:
            # print('RECALCULADO CONSUMO DE ENERGIA')
            # robot_actions_logger.logger.log(f'RECALCULADO CONSUMO DE ENERGIA {distance}')
            self.init_pose = Pose(position_x, position_y)
            # self.init_pose.x = position_x
            # self.init_pose.y = position_y

    def recovery_subroutine(self):
        pass

    def scan_subroutine(self, x: float, y: float, target_yaws_array: list) -> list:
        """
        Creates an action client, sends a goal to the move_base server, and waits for completion.
        """
        timeout: float = 4
        yaw_tolerance = (
            degrees(rospy.get_param("/move_base/DWAPlannerROS/yaw_goal_tolerance"))
            % 360
        )

        if self.cancelled:
            print("scan scan_subroutine cancelled")
            return [0]

        target_yaw = target_yaws_array.pop()
        theta_degrees = target_yaw

        rospy.loginfo("Waiting for move_base action server...")

        goal = PoseStamped()
        goal.header.frame_id = "map"  # The frame in which the goal is specified
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y

        theta_rad = radians(theta_degrees)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta_rad)

        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        rospy.loginfo(
            "Sending goal (X: %.2f, Y: %.2f, Theta: %.2f)..." % (x, y, theta_degrees)
        )
        # self.client1.send_goal(goal)

        start = time.time()
        self.simple_goal_pub.publish(goal)
        # time.sleep(3)

        while abs(self.current_yaw - theta_degrees) > yaw_tolerance:
            if self.cancelled:
                print("cancelled set_target_pose")
                return [0]

            if abs(time.time() - start) > timeout:
                break

        rate: float = self.get_image_similarity()

        if not target_yaws_array:
            rospy.loginfo("Termino el scaneo.....")
            print("el grado de similariddad es: ", rate)
            return [rate]

        return [*self.scan_subroutine(x, y, target_yaws_array.copy()), *[rate]]

    def remove_image(self, path):
        if os.path.exists(path=path):
            os.remove(path=path)
        else:
            print(f"{__name__} el archivo EXISTE {path}")

    def get_image_similarity(self) -> float:
        try:
            if self.current_reference_image_path:
                cv.imwrite(self.camera_image_filepath, self.current_image)
                print(f"{__name__} path:{self.camera_image_filepath}")
                ImgSim = imgsim.ImgSimilarity("efficientnet_b0")

                reference = self.current_reference_image_path
                # reference = self.edit_image(self.current_reference_image_path)
                camera = self.camera_image_filepath
                # self.edit_image(self.camara_image_filepath)
                config = self.user_config.read_data()
                img_width, img_height = 160, 120
                crop_width, crop_height = config["roi"]

                # Calculate the coordinates
                left = (img_width - crop_width) / 2
                top = (img_height - crop_height) / 2
                right = left + crop_width
                bottom = top + crop_height
                # r = ImgSim.similarity((reference, (130, 70, 230, 170)), (camera, None))
                r = ImgSim.similarity(
                    (reference, (left, top, right, bottom)), (camera, None)
                )

                # for p in (source, target):
                #     self.remove_image(p)

                if self.on_calibration:
                    pass
                return r
            return -1
        except Exception as e:
            print("subroutine_wrapper: ", e)
            return -1000

    def save_calibration(
        self, calibration_value: float, calibration_vector: list, map_filename: str, id
    ) -> None:
        self.database = DataBase(
            action="save_calibration",
            data={
                "checkpoint_id": id,
                "value": calibration_value,
                "vector": calibration_vector,
            },
        )
        self.database.action_completed.connect(self.database_task_finished)
        self.database.start()

    def get_calibration(self, current_calibration_vector, id):
        self.database = DataBase(
            action="get_calibration",
            data={
                "pointid": id,
                "current_calibration_vector": current_calibration_vector,
            },
        )
        self.database.action_completed.connect(self.database_task_finished)
        self.database.start()

    def database_task_finished(self, msg, data):
        print("datbase calibration Finished: ", msg)

        if msg == "SuccessGetCalibration":
            mean = data["mean_value"]
            std = data["std_dev_value"]
            loss_func = self.current_rate  # self.get_image_similarity()
            ref = loss_func > (mean - std * 1.5)

            robot_actions_logger.logger.log(
                f"Scaneo finalizados. sim, {loss_func:.4f} std, {std:.4f} mean, {mean:.4f} - good: {ref}"
            )

        self.database.quit()
        self.database.wait()
        self.database = None


# Can do other work here
if __name__ == "__main__":

    def cbdone():
        print("soy un callback de DONE")

    def cbfeeback(x):
        print("SOy un callback de FEEDBACK")
        print(x)

    rospy.init_node("amcl_pose_chatter", anonymous=True)
    ex = PointsScheduler(done_task=cbdone, feedback_task=cbfeeback)
    # ex.dispatch()
    # ex.set_target_pose(0.5, 0.5, 200)
    ex.scan_subroutine()
    # ex.restart()
    # ex.dispatch()
    # rospy.spin()
