import logging
import rospy
from datetime import datetime, date, time
import cv2 as cv

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from database_manager import DataBase

from database_manager import AlertStatus
from internal_storage.tables import Alert



logger = logging.getLogger(__name__)
file_handler = logging.FileHandler("app.log")
file_handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

class AlertGenerator():
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

    def throw_alert(self, message, status, patrol_id, checkpoint_id, map):
        now = datetime.now()
        print('ALERTAAAAA!')
        logger.error(f'{message} {status}')
        
        if self.database and self.database.isRunning():
            return

        alert = {
            'checkpoint_id': str( checkpoint_id ),
            'patrol_id' : str( patrol_id ),
            'message': message,
            'x_position': self.current_pose.position.x,
            'y_position': self.current_pose.position.y,
            'yaw': 0,
            'camera_data': 'camera.jpg',
            'lidar_data': 'no',
            'status': status,
            'date': date.today(),
            'time': time(
                hour= now.hour,
                minute= now.minute,
                second= now.second
                ),
            'map': map,
        }
        self.database = DataBase(action='save_alerts', data={'alerts': [alert]})
        self.database.action_completed.connect(self.database_task_finished)
        self.database.start()

    def database_task_finished(self, x, y):
        print(f'{__name__} alerts save {x}')
        self.database.quit()
        self.database.wait()
        self.database = None

if __name__ == "__main__":
    ex = AlertGenerator()
    ex.throw_alert('Test', -100, '888888' , '777777', 'map.yalm')
    pass
