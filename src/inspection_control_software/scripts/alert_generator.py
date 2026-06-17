import logging
from datetime import date, datetime, time

import cv2 as cv
import rospy
import tf
from cv_bridge import CvBridge
from database_manager import AlertStatus, DataBase
from geometry_msgs.msg import PoseWithCovarianceStamped
from internal_storage.tables import Alert
from sensor_msgs.msg import Image


class AlertGenerator:
    def __init__(self) -> None:
        self.sub_pose = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.callback_pose
        )
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.database = None

    def callback_pose(self, data):
        self.current_pose = data.pose.pose

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = self.cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def throw_alert(self, message, status, patrol_id, checkpoint_id):
        now = datetime.now()
        print("ALERTAAAAA!")

        if self.database and self.database.isRunning():
            return

        alert = {
            "checkpoint_id": str(checkpoint_id),
            "patrol_id": str(patrol_id),
            "message": message,
            "x_position": self.current_pose.position.x,
            "y_position": self.current_pose.position.y,
            "status": status,
            "date": date.today(),
            "time": time(hour=now.hour, minute=now.minute, second=now.second),
        }
        self.database = DataBase(action="save_alerts", data={"alerts": [alert]})
        self.database.action_completed.connect(self.database_task_finished)
        self.database.start()

    def database_task_finished(self, x, y):
        print(f"{__name__} alerts save {x}")
        self.database.quit()
        self.database.wait()
        self.database = None


if __name__ == "__main__":
    ex = AlertGenerator()
    ex.throw_alert("Test", -100, "888888", "777777", "map.yalm")
    pass
