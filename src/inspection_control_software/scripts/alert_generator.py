"""Builds and persists inspection alerts (message + robot pose + camera snapshot)."""
import logging
import os
from datetime import date, datetime, time

import cv2 as cv
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from database_manager import AlertStatus, DataBase
from geometry_msgs.msg import PoseWithCovarianceStamped
from internal_storage.tables import Alert
from sensor_msgs.msg import Image

from utils.thread_manager import ThreadQueueManager


class AlertGenerator:
    """Tracks the robot's current pose and latest camera frame, and saves
    alerts (with a snapshot image) to the database via a queued background
    thread (:class:`~utils.thread_manager.ThreadQueueManager`)."""

    def __init__(self) -> None:
        self.sub_pose = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.callback_pose
        )
        self.cv_image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.database = None
        self._thread_manager = ThreadQueueManager()

    def callback_pose(self, data):
        """ROS subscriber callback: track the robot's current AMCL pose."""
        self.current_pose = data.pose.pose

    def image_callback(self, msg):
        """ROS subscriber callback: keep the latest camera frame and its
        would-be alert-image filename ready for :meth:`throw_alert`."""
        try:
            # Convert ROS Image message to OpenCV image
            self.output_dir = "./alert_images"
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Construct a unique filename
            timestamp_int = datetime.now()
            self.filename = os.path.join(self.output_dir, f"image_{timestamp_int}.png")



        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")

    def throw_alert(self, message, status, patrol_id, checkpoint_id, map_filepath):
        """Save the current camera frame to disk and queue a database save
        for a new alert row referencing it."""
        now = datetime.now()
        print("ALERTAAAAA!")

        if self.database and self.database.isRunning():
            return


        # Save the image to disk
        cv.imwrite(self.filename, self.cv_image)

        alert = {
            "checkpoint_id": str(checkpoint_id),
            "patrol_id": str(patrol_id),
            "message": message,
            "x_position": self.current_pose.position.x,
            "y_position": self.current_pose.position.y,
            "status": status,
            "date": date.today(),
            "time": time(hour=now.hour, minute=now.minute, second=now.second),
            "map_filepath": map_filepath,
            "image_filepath": self.filename,
            "image": self.cv_image,
        }
        # self.database = DataBase(action="save_alerts", data={"alerts": [alert]})
        database = DataBase(action="save_alerts", data={"alerts": [alert]})
        self._thread_manager.enqueue(database)
        # self.database.action_completed.connect(self.database_task_finished)
        # self.database.start()

    def database_task_finished(self, x, y):
        """Unused completion callback (never connected; ``save_alerts`` goes
        through the thread queue manager instead)."""
        print(f"{__name__} alerts save {x}")
        self.database.quit()
        self.database.wait()
        self.database = None


if __name__ == "__main__":
    ex = AlertGenerator()
    ex.throw_alert("Test", -100, "888888", "777777", "map.yalm")
    pass
