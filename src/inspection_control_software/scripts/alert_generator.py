import logging
import rospy


from geometry_msgs.msg import PoseWithCovarianceStamped


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

    def callback_pose(self, data):
        self.current_pose = data.pose.pose

    def throw_alert(self, name, status):
        print('ALERTAAAAA!')
        logger.error(f'{name} {status}')
