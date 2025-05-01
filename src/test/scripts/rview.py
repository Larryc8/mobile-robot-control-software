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


## Finally import the RViz bindings themselves.
from rviz import bindings as rviz


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
    def __init__(self, configfile: str, parent=None):
        super().__init__(parent)  # Pasa `parent` al constructor de QWidget
        # QWidget.__init__(self)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

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
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        # self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget(self.frame)

        self.setLayout(layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    t = MyViz()
    t.show()

    t.t2 = MyViz()
    t.t2.show()
    sys.exit(app.exec())
