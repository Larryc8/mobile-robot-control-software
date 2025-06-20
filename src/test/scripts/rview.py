#!/usr/bin/env python

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
# import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from python_qt_binding.QtWidgets import *
import rospy


## Finally import the RViz bindings themselves.
from rviz import bindings as rviz
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# try:
# rospy.get_master().getPid()
# except:
# failed = True


## The MyViz class is the main container widget.
class MyViz(QWidget):
    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self, configfile: str, parent=None, mini=False):
        super().__init__(parent)  # Pasa `parent` al constructor de QWidget
        # QWidget.__init__(self)
        self.past_point = QPoint(0, 0)
        self.frame_position = QPoint(0, 0)
        self.followRobot = False
        self.minHeight = self.height()
        self.minWidth = self.width()
        self.resize = 145
        # rospy.init_node('harolsito')

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()
        # self.m = rviz.VisualizationManager()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath("")

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, configfile)
        self.frame.load(config)

        self.setWindowTitle(config.mapGetChild("Title").getValue())

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        # self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()
        self.view_man = self.manager.getViewManager()
        # print('view: ', self.view_man.getCurrent().subProp( "Angle" ).setValue(-5), self.view_man.getNumViews())
        rospy.Subscriber("/odom", Odometry, self.callback)

        # self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget(self.frame)
        self.setLayout(layout)

    def callback(self, msg):
        orientation = msg.pose.pose.orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        present_point = QPoint(x, y)
        delta_movement = present_point - self.past_point
        self.past_point = present_point

        if self.followRobot:
            (roll, pitch, yaw) = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            self.view_man.getCurrent().subProp("Angle").setValue(yaw - 1.55678)
            # self.view_man.getCurrent().subProp( "Global Options" ).subProp( "Fixed Frame" ).setValue('odom')
            self.view_man.getCurrent().subProp("X").setValue(
                self.view_man.getCurrent().subProp("X").getValue() + delta_movement.x() 
            )
            self.view_man.getCurrent().subProp("Y").setValue(
                self.view_man.getCurrent().subProp("Y").getValue() + delta_movement.y()
            )
            # self.view_man.getCurrent().subProp( "Y" ).setValue(self.frame_position.y())

            # if self.width() > self.height():
            #     delta_movement =  delta_movement*self.width()/self.height()
            # else:
            #     delta_movement = delta_movement*(self.width()/self.minWidth)
            self.frame_position = self.frame_position + delta_movement

        # print(
        #     "RVIZ",
        #     self.view_man.getCurrent().subProp("Y").getValue(),
        #     self.view_man.getCurrent().subProp("Y").getValue(),
        #     delta_movement.x(),
        #     delta_movement.y()
        # )

    def setUp(self, prop: str, value):  # SCale 562
        if prop == "globalframe":
            self.manager.setFixedFrame(value)
            if value == 'map':
                self.view_man.getCurrent().subProp("Angle").setValue(0)
        if prop == "followrobot":
            self.followRobot = value
            self.view_man.getCurrent().subProp("Scale").setValue(self.resize)
            self.manager.setFixedFrame("odom")
        if prop == "scale":
            self.view_man.getCurrent().subProp("Scale").setValue(value)
        if prop == "reset":
            for i in range(12):
                display = self.manager.getRootDisplayGroup().getDisplayAt(i)
                if display:
                    display.reset()
                # display.subProp( "Enabled" )#.setValue(False)
                # display.subProp("Line Style")  # .setValue(True)
            pass

    def resizeEvent(self, event):
        if self.width() > self.height():
            # self.view_man.getCurrent().subProp( "Scale" ).setValue(145*self.width()/765)
            # self.resize = 145*self.width()/self.height()
            pass
        else:
            # self.view_man.getCurrent().subProp( "Scale" ).setValue(145*self.width()/self.minWidth)
            # self.resize = 145*self.width()/self.minWidth
            pass

        if self.followRobot:
            self.view_man.getCurrent().subProp("Scale").setValue(self.resize)
        super().resizeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = MyViz()
    t.show()

    t.t2 = MyViz()
    t.t2.show()
    sys.exit(app.exec())
