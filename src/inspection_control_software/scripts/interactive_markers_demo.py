#!/usr/bin/env python3

import copy
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
from tf.transformations import quaternion_from_euler
from utils.patrol import (
    MarkerActionTriggered,
    PatrolEndState,
    operationMode,
    userOperation,
)
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class InteractiveMarkerDemo(QObject):
    points_changed = pyqtSignal(dict)
    action_triggered = pyqtSignal(MarkerActionTriggered, Marker)

    def __init__(self):
        super().__init__()
        # Initialize the interactive marker server
        self.server = InteractiveMarkerServer("interactive_markers_demo")

        self.__path = {}
        self.__isUpdated = False
        self.__action_triggered = {
            "owner": None,
            "action": None,
        }
        self.markers_name = []
        self.__interactive_markers = {}
        self.__database = None
        self.__user_operation = None

        # Create a menu handler
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Mostrar imagen", callback=self.process_feedback)
        self.menu_handler.insert("Establecer Home", callback=self.process_feedback)

        # Create a sub-menu
        sub_menu_handle = self.menu_handler.insert("Actions")
        self.menu_handler.insert(
            "Reset Position", parent=sub_menu_handle, callback=self.process_feedback
        )
        self.menu_handler.insert(
            "Log Status", parent=sub_menu_handle, callback=self.process_feedback
        )
        self.menu_handler.insert("Delete Marker", callback=self.process_feedback)

        # Create our interactive marker
        # self.make_6dof_marker("demo_marker", "Marker", Point(0, 0, 0))
        # self.make_6dof_marker("demo_marker2", "Marker", Point(0, 1, 0))
        # self.make_6dof_marker("demo_marker3", "Marker", Point(1, 1, 0))

        # Apply changes to the server
        self.server.applyChanges()

        # Subscribe to external goal topics
        # self.goal_sub = rospy.Subscriber("simple_action_goal", PoseStamped, self.external_goal_callback)
        self.move_base_goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.external_goal_callback
        )

        rospy.loginfo("Interactive Marker Server initialized.")

        self.timer = QTimer(self)

        # 3. Connect the timer's 'timeout' signal to a function
        self.timer.timeout.connect(self.update_widgets)

        # 4. Start the timer (interval in milliseconds)
        # 100ms means it will trigger 10 times per second
        self.timer.start(100)

    def set_user_operation(self, user_operation):
        self.__user_operation = user_operation
        if (
            self.__user_operation == userOperation.CREATEMAP
            or self.__user_operation == userOperation.LOADMAP
        ):
            self.__interactive_markers.clear()
            self.server.clear()
            self.server.applyChanges()

        if self.__user_operation == userOperation.LOADMAP:
            self.__isUpdated = True

    def bulk_create_markers(self, points):
        for point in points.values():
            yaw = point["yaw"]
            quat_tuple = quaternion_from_euler(0, 0, yaw)
            quat_msg = Quaternion()
            quat_msg.x = quat_tuple[0]
            quat_msg.y = quat_tuple[1]
            quat_msg.z = quat_tuple[2]
            quat_msg.w = quat_tuple[3]
            self.make_6dof_marker(
                point["id"],
                "Marker",
                Pose(position=Point(point["x"], point["y"], 0), orientation=quat_msg),
                img=point.get("image"),
            )
        self.server.applyChanges()

    def send_database_action(self, map_file):
        if self.__database and self.__database.isRunning():
            # self.actions_queue.append((action, data))
            return

        self.database = DataBase(action="get_points", data={"map_file": map_file})
        self.database.action_completed.connect(self.load_markers_from_database)
        self.database.start()

    def load_markers_from_database(self, msg, data):
        points = data.get("points")
        # Load markers from the database
        self.bulk_create_markers(points)

    def update_markers(self, current_poin_id, next_point_id, point_state):
        for p in self.__interactive_markers.values():
            if p["owner"].name == current_poin_id:
                self.update_color(current_poin_id, 1.0, 0.8, 0.0, 0.9)
                # self.update_color(current_poin_id, 1.0, 0.5, 0.3, 0.7)
            else:
                # self.update_color(p.name, 1.0, 0.5, 0.3, 0.9)
                self.update_color(p["owner"].name, 1.0, 0.5, 0.3, 0.8)

    def update_widgets(self):
        """This function runs every time the timer 'ticks'"""
        if self.__isUpdated:
            self.__path = {}

            for marker in self.__interactive_markers.values():
                # Get Euler angles in radians (roll, pitch, yaw)
                p = marker["owner"]
                quat_list = [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat_list)

                self.__path[str(p.name)] = {
                    "x_meters": p.pose.position.x,  # * self.resolution,
                    "y_meters": p.pose.position.y,  # * self.resolution,
                    "yaw_degrees": 0,
                    "yaw": yaw,
                    "checked": False,
                    "mapfile": "NA",
                    "type": 0,
                    "gui_yaw": 0,  # point.get("gui_yaw"),
                    "image": p.controls[0]
                    .markers[0]
                    .mesh_resource,  # point.get("image"),
                    "aruco_pose_vector": None,
                    "is_home": marker["is_home"],
                }

            self.points_changed.emit(self.__path)

            self.__isUpdated = False

        if self.__action_triggered["owner"] is not None:
            self.action_triggered.emit(
                self.__action_triggered["action"],
                self.__action_triggered["owner"].controls[0].markers[0],
            )
            self.__action_triggered = {
                "owner": None,
                "action": None,
            }

    def process_feedback(self, feedback):
        """Callback to handle marker feedback."""
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(
                f"{s}: pose changed to {feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f}, {feedback.pose.position.z:.2f}"
            )

        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(f"{s}: button click!")

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(f"{s}: menu item {feedback.menu_entry_id} clicked.")

            # Handle specific menu items
            if feedback.menu_entry_id == 1:  # Say Hello
                rospy.loginfo(">>> HELLO FROM INTERACTIVE MARKER! <<<")
                self.__action_triggered = {
                    "owner": self.__interactive_markers[feedback.marker_name]["owner"],
                    "action": MarkerActionTriggered.HELLO,
                }
            elif feedback.menu_entry_id == 2:  # Set as Home
                rospy.loginfo(">>> GOODBYE FROM INTERACTIVE MARKER! <<<")
                self.update_label(feedback.marker_name, "Home")
                for name, marker in self.__interactive_markers.items():
                    if name != feedback.marker_name:
                        self.update_label(name, "Marker")
                        self.__interactive_markers[name]["is_home"] = False
                self.__interactive_markers[feedback.marker_name]["is_home"] = True
                self.__isUpdated = True

            elif (
                feedback.menu_entry_id == 4
            ):  # Reset Position (ID 3 is the 'Actions' folder)
                rospy.loginfo("Resetting position...")
                feedback.pose.position.x = 0
                feedback.pose.position.y = 0
                feedback.pose.position.z = 0
                feedback.pose.orientation.x = 0
                feedback.pose.orientation.y = 0
                feedback.pose.orientation.z = 0
                feedback.pose.orientation.w = 1
                self.server.setPose(feedback.marker_name, feedback.pose)
                self.server.applyChanges()
            elif feedback.menu_entry_id == 5:  # Log Status
                rospy.loginfo(f"Current Pose: {feedback.pose}")
            elif feedback.menu_entry_id == 6:  # Delete Marker
                rospy.loginfo(f"Deleting marker '{feedback.marker_name}'")
                self.server.erase(feedback.marker_name)
                self.__interactive_markers.pop(feedback.marker_name)
                print(f"Deleted marker '{feedback.marker_name}'")
                print(f"Remaining markers: {list(self.__interactive_markers.keys())}")
                self.__isUpdated = True
                self.server.applyChanges()

    def external_goal_callback(self, msg):
        """Updates the marker pose based on an external PoseStamped message."""
        rospy.loginfo(
            f"Received external goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}). Updating marker."
        )
        self.make_6dof_marker(f"demo_marker{random.random()}", "Marker", msg.pose)
        self.__isUpdated = True
        # self.server.setPose("demo_marker", msg.pose)
        # self.server.applyChanges()
        self.server.applyChanges()
        # print(self.__interactive_markers)

    def update_color(self, name, r, g, b, a=1):
        # 2. Access the LOCAL copy, not the server
        if name in self.__interactive_markers:
            int_marker = self.__interactive_markers[name]["owner"]

            # Navigate the nested message structure
            # (Control 0 -> Marker 0)
            int_marker.controls[0].markers[0].color.r = r
            int_marker.controls[0].markers[0].color.g = g
            int_marker.controls[0].markers[0].color.b = b
            int_marker.controls[0].markers[0].color.a = a

            # 3. Re-insert the updated object to the server
            self.server.insert(int_marker)
            self.server.applyChanges()
            rospy.loginfo(f"Changed {name} color to RGB({r},{g},{b})")

    def update_label(self, name, label):
        if name in self.__interactive_markers:
            int_marker = self.__interactive_markers[name]["owner"]
            int_marker.controls[0].markers[1].text = label
            if not label == "Marker":
                int_marker.controls[0].markers[0].type = Marker.CUBE
            else:
                int_marker.controls[0].markers[0].type = Marker.ARROW

            # 3. Re-insert the updated object to the server
            self.server.insert(int_marker)
            self.server.applyChanges()

            rospy.loginfo(f"Updated {name} label to {label}")

    def make_6dof_marker(self, name, description, position, img=""):
        """Creates a marker with a custom floating text label."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        # int_marker.pose.position = position
        int_marker.pose = position
        int_marker.scale = 0.5  # Controls the size of the interaction handles
        int_marker.name = name

        # We leave int_marker.description empty to hide the default label
        int_marker.description = ""

        # 1. Create a control for the visuals
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON

        # 2. Define the Cylinder (The physical object)
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.3, a=0.8)
        box_marker.scale = Vector3(x=0.4, y=0.15, z=0.05)
        box_marker.mesh_resource = img
        control.markers.append(box_marker)

        # 3. Define the Floating Text (The "Description")
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = description  # Use the passed-in description string
        text_marker.color = ColorRGBA(
            r=0, g=0.0, b=0.0, a=1.0
        )  # ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # Solid white

        # Adjust text size here (0.15 is usually readable)
        text_marker.scale.z = 0.15

        # --- OFFSET THE TEXT ---
        # This places the text slightly above the cylinder
        text_marker.pose.position.z = 0.2

        control.markers.append(text_marker)

        # Add the combined control to the interactive marker
        int_marker.controls.append(control)

        # Add to server
        self.server.insert(int_marker, self.process_feedback)
        self.__interactive_markers[int_marker.name] = {
            "is_home": False,
            "owner": int_marker,
        }

        # Apply menu to this marker
        self.menu_handler.apply(self.server, int_marker.name)

    # def make_6dof_marker(self, name, description, position):
    #     """Creates a 6-DOF interactive marker with a visual representation."""
    #     int_marker = InteractiveMarker()
    #     int_marker.header.frame_id = "map" # Usually map or base_link
    #     int_marker.pose.position = position
    #     int_marker.scale = 0.4
    #     int_marker.name = name
    #     int_marker.description = description

    #     # 1. Add a visual representation (a box)
    #     # This control allows clicking because its interaction mode is BUTTON
    #     box_control = InteractiveMarkerControl()
    #     box_control.always_visible = True
    #     box_control.interaction_mode = InteractiveMarkerControl.BUTTON

    #     # Define the visual marker
    #     box_marker = Marker()
    #     box_marker.type = Marker.CYLINDER
    #     box_marker.color = ColorRGBA(r=1, g=0.5, b=0.3, a=0.9)
    #     box_marker.scale = Vector3(x=0.3, y=0.3, z=0.01)

    #     box_control.markers.append(box_marker)
    #     int_marker.controls.append(box_control)

    #     # 2. Add movement controls (X, Y, Z translation and rotation)
    #     # Move X
    #     # control = InteractiveMarkerControl()
    #     # control.orientation.w = 1
    #     # control.orientation.x = 1
    #     # control.name = "move_x"
    #     # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # # Rotate X
    #     # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #     # control.name = "rotate_x"
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # # Move Y
    #     # control.orientation.x = 0
    #     # control.orientation.y = 1
    #     # control.name = "move_y"
    #     # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # # Rotate Y
    #     # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #     # control.name = "rotate_y"
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # # Move Z
    #     # control.orientation.y = 0
    #     # control.orientation.z = 1
    #     # control.name = "move_z"
    #     # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # # Rotate Z
    #     # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #     # control.name = "rotate_z"
    #     # int_marker.controls.append(copy.deepcopy(control))

    #     # Add to server
    #     self.server.insert(int_marker, self.process_feedback)

    #     # Apply menu to this marker
    #     self.menu_handler.apply(self.server, int_marker.name)
    #


if __name__ == "__main__":
    rospy.init_node("interactive_markers_demo_node")
    demo = InteractiveMarkerDemo()
    rospy.spin()
