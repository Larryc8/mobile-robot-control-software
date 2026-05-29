#!/usr/bin/env python3

import random
from enum import Enum

import rospy
import tf
from database_manager import DataBase
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils.patrol import MarkerActionTriggered, userOperation
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


# =====================================================================
# 1. SINGLE RESPONSIBILITY: Marker Component Factory
# =====================================================================
class MarkerFactory:
    """Responsible solely for building ROS Visualization Markers and Controls."""

    @staticmethod
    def create_arrow_mesh(img_path: str) -> Marker:
        marker = Marker()
        marker.type = Marker.ARROW
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.3, a=0.8)
        marker.scale = Vector3(x=0.4, y=0.15, z=0.05)
        marker.mesh_resource = img_path
        return marker

    @staticmethod
    def create_text_label(description: str) -> Marker:
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = description
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        marker.scale.z = 0.15
        marker.pose.position.z = 0.2
        return marker

    @classmethod
    def build_interactive_marker(
        cls, name: str, description: str, pose: Pose, img: str = ""
    ) -> InteractiveMarker:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = pose
        int_marker.scale = 0.5
        int_marker.name = name
        int_marker.description = ""

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON

        control.markers.append(cls.create_arrow_mesh(img))
        control.markers.append(cls.create_text_label(description))

        int_marker.controls.append(control)
        return int_marker


# =====================================================================
# 2. SINGLE RESPONSIBILITY: State Tracking (Repository Pattern)
# =====================================================================
class MarkerRepository:
    """Responsible for maintaining the runtime state of loaded markers."""

    def __init__(self):
        self._markers = {}

    def clear(self):
        self._markers.clear()

    def add(self, name: str, int_marker: InteractiveMarker, is_home: bool = False):
        self._markers[name] = {"is_home": is_home, "owner": int_marker}

    def remove(self, name: str):
        self._markers.pop(name, None)

    def exists(self, name: str) -> bool:
        return name in self._markers

    def get_all(self):
        return self._markers.values()

    def get(self, name: str):
        return self._markers.get(name)

    def get_names(self):
        return list(self._markers.keys())

    def update_home_status(self, home_marker_name: str):
        for name, data in self._markers.items():
            if name == home_marker_name:
                data["is_home"] = True
            else:
                data["is_home"] = False


# =====================================================================
# 3. OPEN/CLOSED PRINCIPLE: Extensible Action Dispatcher
# =====================================================================
class MenuEntry(Enum):
    SHOW_IMAGE = 1
    SET_HOME = 2
    RESET_POSITION = 4
    LOG_STATUS = 5
    DELETE_MARKER = 6


class InteractiveMarkerDemo(QObject):
    points_changed = pyqtSignal(dict)
    action_triggered = pyqtSignal(MarkerActionTriggered, Marker)

    def __init__(self, server_name: str = "interactive_markers_demo"):
        super().__init__()
        self.server = InteractiveMarkerServer(server_name)
        self.repository = MarkerRepository()
        self.menu_handler = MenuHandler()

        self.__is_updated = False
        self.__database = None
        self.__user_operation = None

        self._initialize_menu()
        self._initialize_subscribers()
        self._initialize_timer()

        rospy.loginfo("Interactive Marker Server initialized.")

    def _initialize_menu(self):
        self.menu_handler.insert("Mostrar imagen", callback=self.process_feedback)
        self.menu_handler.insert("Establecer Home", callback=self.process_feedback)

        sub_menu = self.menu_handler.insert("Actions")
        self.menu_handler.insert(
            "Reset Position", parent=sub_menu, callback=self.process_feedback
        )
        self.menu_handler.insert(
            "Log Status", parent=sub_menu, callback=self.process_feedback
        )
        self.menu_handler.insert("Delete Marker", callback=self.process_feedback)

    def _initialize_subscribers(self):
        self.move_base_goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.external_goal_callback
        )

    def _initialize_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(100)

    def set_user_operation(self, user_operation):
        self.__user_operation = user_operation
        if user_operation in [userOperation.CREATEMAP, userOperation.LOADMAP]:
            self.repository.clear()
            self.server.clear()
            self.server.applyChanges()

        if user_operation == userOperation.LOADMAP:
            self.__is_updated = True

    def bulk_create_markers(self, points: dict):
        for point in points.values():
            quat_tuple = quaternion_from_euler(0, 0, point["yaw"])
            pose = Pose(
                position=Point(point["x"], point["y"], 0),
                orientation=Quaternion(*quat_tuple),
            )
            self.register_new_marker(
                point["id"], "Marker", pose, img=point.get("image")
            )
        self.server.applyChanges()

    def register_new_marker(
        self, name: str, description: str, pose: Pose, img: str = ""
    ) -> InteractiveMarker:
        int_marker = MarkerFactory.build_interactive_marker(
            name, description, pose, img
        )

        self.server.insert(int_marker, self.process_feedback)
        self.repository.add(name, int_marker)
        self.menu_handler.apply(self.server, name)
        return int_marker

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        msg_prefix = f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(
                f"{msg_prefix}: pose changed to {feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f}"
            )
            return

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(f"{msg_prefix}: menu item {feedback.menu_entry_id} clicked.")
            try:
                action = MenuEntry(feedback.menu_entry_id)
                self._handle_menu_action(action, feedback)
            except ValueError:
                rospy.logwarn(
                    f"Unknown Menu Entry ID received: {feedback.menu_entry_id}"
                )

    def _handle_menu_action(
        self, action: MenuEntry, feedback: InteractiveMarkerFeedback
    ):
        """Dispatches operational strategies based on menu item choice."""
        if action == MenuEntry.SHOW_IMAGE:
            rospy.loginfo(">>> HELLO FROM INTERACTIVE MARKER! <<<")
            marker_data = self.repository.get(feedback.marker_name)
            if marker_data:
                self.action_triggered.emit(
                    MarkerActionTriggered.HELLO,
                    marker_data["owner"].controls[0].markers[0],
                )

        elif action == MenuEntry.SET_HOME:
            if self._db_is_busy():
                return
            rospy.loginfo(">>> SETTING MARKER AS HOME <<<")
            self.repository.update_home_status(feedback.marker_name)
            self.update_label(feedback.marker_name, "Home")

            for name in self.repository.get_names():
                if name != feedback.marker_name:
                    self.update_label(name, "Marker")

            self._execute_db_transaction(
                "update_point", {feedback.marker_name: {"is_home": True}}
            )

        elif action == MenuEntry.RESET_POSITION:
            rospy.loginfo("Resetting position...")
            blank_pose = Pose(orientation=Quaternion(w=1.0))
            self.server.setPose(feedback.marker_name, blank_pose)
            self.server.applyChanges()

        elif action == MenuEntry.LOG_STATUS:
            rospy.loginfo(f"Current Pose: {feedback.pose}")

        elif action == MenuEntry.DELETE_MARKER:
            if self._db_is_busy():
                return
            rospy.loginfo(f"Deleting marker '{feedback.marker_name}'")
            self.server.erase(feedback.marker_name)
            self.repository.remove(feedback.marker_name)
            self.server.applyChanges()

            self._execute_db_transaction(
                "update_point", {feedback.marker_name: {"enabled": False}}
            )

    def _db_is_busy(self) -> bool:
        return self.__database is not None and self.__database.isRunning()

    def _execute_db_transaction(self, action: str, data: dict, callback=None):
        if self._db_is_busy():
            return
        callback = callback or self.db_operation_finished
        self.__database = DataBase(action=action, data=data)
        self.__database.action_completed.connect(callback)
        self.__database.start()

    def send_database_action(self, map_file: str):
        self._execute_db_transaction(
            "get_points", {"map_file": map_file}, self.load_markers_from_database
        )

    def load_markers_from_database(self, msg, data: dict):
        self._cleanup_db_thread()
        self.bulk_create_markers(data.get("points", {}))

    def db_operation_finished(self, action, data):
        self._cleanup_db_thread()
        self.__is_updated = True
        self.server.applyChanges()

    def _cleanup_db_thread(self):
        if self.__database:
            self.__database.quit()
            self.__database.wait()
            self.__database = None

    def external_goal_callback(self, msg: PoseStamped):
        rospy.loginfo(
            f"Received external goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})."
        )
        generated_name = f"demo_marker_{random.random()}"
        int_marker = self.register_new_marker(generated_name, "Marker", msg.pose)

        formatted_data = self.format_marker_payload(
            generated_name, {"is_home": False, "owner": int_marker}
        )
        self._execute_db_transaction("save_point", formatted_data)

    # =====================================================================
    # Formatting & Widget Synchronization
    # =====================================================================
    def update_widgets(self):
        if not self.__is_updated:
            return

        path_payload = {}
        for name, marker_data in self.repository._markers.items():
            path_payload.update(self.format_marker_payload(name, marker_data))

        self.points_changed.emit(path_payload)
        self.__is_updated = False

    def format_marker_payload(self, name: str, marker_data: dict) -> dict:
        p = marker_data["owner"]
        quat = [
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quat)

        return {
            str(name): {
                "x_meters": p.pose.position.x,
                "y_meters": p.pose.position.y,
                "yaw": yaw,
                "mapfile": "NA",
                "image_path": p.controls[0].markers[0].mesh_resource,
                "enabled": True,
                "is_home": marker_data["is_home"],
            }
        }

    def update_markers(self, current_poin_id, next_point_id, point_state):
        for p in self.repository.get_all():
            if p["owner"].name == current_poin_id:
                self.update_color(current_poin_id, 1.0, 0.8, 0.0, 0.9)
                # self.update_color(current_poin_id, 1.0, 0.5, 0.3, 0.7)
            else:
                # self.update_color(p.name, 1.0, 0.5, 0.3, 0.9)
                self.update_color(p["owner"].name, 1.0, 0.5, 0.3, 0.8)

    def update_color(self, name: str, r: float, g: float, b: float, a: float = 1.0):
        marker_data = self.repository.get(name)
        if marker_data:
            int_marker = marker_data["owner"]
            int_marker.controls[0].markers[0].color = ColorRGBA(r=r, g=g, b=b, a=a)
            self.server.insert(int_marker)
            self.server.applyChanges()

    def update_label(self, name: str, label: str):
        marker_data = self.repository.get(name)
        if marker_data:
            int_marker = marker_data["owner"]
            int_marker.controls[0].markers[1].text = label
            int_marker.controls[0].markers[0].type = (
                Marker.CUBE if label != "Marker" else Marker.ARROW
            )
            self.server.insert(int_marker)


if __name__ == "__main__":
    rospy.init_node("interactive_markers_demo_node")
    demo = InteractiveMarkerDemo()
    rospy.spin()
