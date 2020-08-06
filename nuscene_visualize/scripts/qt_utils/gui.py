from PyQt5 import QtCore, QtGui, QtWidgets
import os

class Ui_Dialog(object):
    def __init__(self, ros_interface):
        self.ros_interface = ros_interface
        self.reset()

    def reject(self):
        self.quit = True
        self.Dialog.reject()

    def reset(self):
        self.quit = False
        self.text_set = False


    def img_index_changed(self):
        value = self.imageIndex.value()
        self.ros_interface.publish_index(value)

    def stop_changed(self):
        bool_value = self.checkBox_2.isChecked()
        self.ros_interface.publish_stop_bool(bool_value)

    def pause_changed(self):
        bool_value = self.checkBox_3.isChecked()
        self.ros_interface.publish_pause_bool(bool_value)


    def setupUi(self, Dialog):
        self.Dialog = Dialog
        Dialog.setObjectName("Dialog")
        Dialog.resize(400, 225)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(310, 160, 81, 241))
        self.buttonBox.setOrientation(QtCore.Qt.Vertical)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.imageIndex = QtWidgets.QSpinBox(Dialog)
        self.imageIndex.setGeometry(QtCore.QRect(80, 40, 81, 41))
        self.imageIndex.setMaximum(1000000)
        self.imageIndex.setObjectName("imageIndex")
        self.imageIndex.valueChanged.connect(self.img_index_changed)

        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(40, 50, 51, 21))
        self.label.setObjectName("label")

        self.checkBox_2 = QtWidgets.QCheckBox(Dialog)
        self.checkBox_2.setGeometry(QtCore.QRect(30, 150, 131, 41))
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_2.setChecked(True)
        self.checkBox_2.toggled.connect(self.stop_changed)

        self.checkBox_3 = QtWidgets.QCheckBox(Dialog)
        self.checkBox_3.setGeometry(QtCore.QRect(110, 150, 161, 41))
        self.checkBox_3.setChecked(False)
        self.checkBox_3.setObjectName("checkBox_3")
        self.checkBox_3.toggled.connect(self.pause_changed)

        self.retranslateUi(Dialog)
        self.buttonBox.rejected.connect(self.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label.setText(_translate("Dialog", "index"))
        self.checkBox_2.setText(_translate("Dialog", "Stop"))
        self.checkBox_3.setText(_translate("Dialog", "Pause(for sequence)"))
