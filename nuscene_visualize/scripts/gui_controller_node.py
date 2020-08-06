#!/usr/bin/env python
"""
    This script launchs a control panel for GUI interaction with Nuscenes visualization.

    User manual:

    index: integer selection, notice do not overflow the index number (764 for v1.0-trainval)

    Stop: stop any data loading or processing of the visualization node.
    
    Pause: prevent pointer of the sequantial data stream from increasing, keep the current scene.

    Cancel: quit.
"""
import rospy 
import sys
from PyQt5 import QtWidgets
from std_msgs.msg import Bool, String, Int32, Float32
from qt_utils.gui import Ui_Dialog

class RosInterface(object):
    def __init__(self):
        super(RosInterface, self).__init__()
        self.index_pub = rospy.Publisher("/nuscenes/control/index", Int32, queue_size=1) 
        self.pause_pub = rospy.Publisher("/nuscenes/control/pause", Bool, queue_size=1)
        self.stop_pub  = rospy.Publisher("/nuscenes/control/stop", Bool, queue_size=1)

    def publish_index(self, index):
        self.index_pub.publish(Int32(index))

    def publish_stop_bool(self, boolean):
        self.stop_pub.publish(Bool(boolean))

    def publish_pause_bool(self, boolean):
        self.pause_pub.publish(Bool(boolean))


class GUIControllerNode:
    def __init__(self):
        rospy.init_node("GUI_controller_node")
        rospy.loginfo("Starting GUI_controller_node.")
        self.qt_processor = Ui_Dialog(RosInterface())

    def start(self):
        while not rospy.is_shutdown() and not self.qt_processor.quit:
            app = QtWidgets.QApplication(sys.argv)
            diag = QtWidgets.QDialog()
            self.qt_processor.setupUi(diag)
            diag.show()
            app.exec_()
            app = None



if __name__ == "__main__":
    ros_node = GUIControllerNode()
    ros_node.start()