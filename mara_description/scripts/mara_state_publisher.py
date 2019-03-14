#!/usr/bin/python3
# -*- coding: utf-8 -*-

from time import sleep

import rclpy
import sys

from sensor_msgs.msg import JointState

#Qt5
from PyQt5.QtWidgets import (QWidget, QSlider, QGridLayout, QPushButton,
    QLabel, QApplication)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPixmap

# We do not recommend this style as ROS 2 provides timers for this purpose,
# and it is recommended that all nodes call a variation of spin.
# This example is only included for completeness because it is similar to examples in ROS 1.
# For periodic publication please see the other examples using timers.
class DoubleSlider(QSlider):

    # create our our signal that we can connect to if necessary
    doubleValueChanged = pyqtSignal(float)

    def __init__(self, decimals=3, *args, **kargs):
        super(DoubleSlider, self).__init__( Qt.Horizontal, **kargs)
        self._multi = 10 ** decimals

        self.valueChanged.connect(self.emitDoubleValueChanged)

    def emitDoubleValueChanged(self):
        value = float(super(DoubleSlider, self).value())/self._multi
        self.doubleValueChanged.emit(value)

    def value(self):
        return float(super(DoubleSlider, self).value()) / self._multi

    def setMinimum(self, value):
        return super(DoubleSlider, self).setMinimum(value * self._multi)

    def setMaximum(self, value):
        return super(DoubleSlider, self).setMaximum(value * self._multi)

    def setSingleStep(self, value):
        return super(DoubleSlider, self).setSingleStep(value * self._multi)

    def singleStep(self):
        return float(super(DoubleSlider, self).singleStep()) / self._multi

    def setValue(self, value):
        super(DoubleSlider, self).setValue(int(value * self._multi))

class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def sendJointStates(self):
        msg = JointState()
        msg.name.append("motor1")
        msg.position.append(self.motor1)
        msg.name.append("motor2")
        msg.position.append(self.motor2)
        msg.name.append("motor3")
        msg.position.append(self.motor3)
        msg.name.append("motor4")
        msg.position.append(self.motor4)
        msg.name.append("motor5")
        msg.position.append(self.motor5)
        msg.name.append("motor6")
        msg.position.append(self.motor6)
        self.publisher.publish(msg)

    def initUI(self):

      rclpy.init(args=sys.argv)

      node = rclpy.create_node('joint_states_publisher_dh_robotics')

      self.publisher = node.create_publisher(JointState, '/joint_states')

      grid = QGridLayout()
      self.setLayout(grid)

      labelMotor1 = QLabel("Motor1")
      self.labelMotor1_value = QLabel("")
      labelMotor2 = QLabel("Motor2")
      self.labelMotor2_value = QLabel("")
      labelMotor3 = QLabel("Motor3")
      self.labelMotor3_value = QLabel("")
      labelMotor4 = QLabel("Motor4")
      self.labelMotor4_value = QLabel("")
      labelMotor5 = QLabel("Motor5")
      self.labelMotor5_value = QLabel("")
      labelMotor6 = QLabel("Motor6")
      self.labelMotor6_value = QLabel("")

      sldMotor1 = DoubleSlider(decimals=3)
      sldMotor1.setMinimum(-3.1416)
      sldMotor1.setMaximum(3.1416)
      sldMotor1.setFocusPolicy(Qt.NoFocus)
      sldMotor1.setGeometry(30, 40, 100, 30)
      sldMotor1.doubleValueChanged[float].connect(self.changeValueMotor1)

      sldMotor2 = DoubleSlider(decimals=3)
      sldMotor2.setMinimum(-3.1416)
      sldMotor2.setMaximum(3.1416)
      sldMotor2.setFocusPolicy(Qt.NoFocus)
      sldMotor2.setGeometry(30, 40, 100, 30)
      sldMotor2.doubleValueChanged[float].connect(self.changeValueMotor2)

      sldMotor3= DoubleSlider(decimals=3)
      sldMotor3.setMinimum(-3.1416)
      sldMotor3.setMaximum(3.1416)
      sldMotor3.setFocusPolicy(Qt.NoFocus)
      sldMotor3.setGeometry(30, 40, 100, 30)
      sldMotor3.doubleValueChanged[float].connect(self.changeValueMotor3)

      sldMotor4 = DoubleSlider(decimals=3)
      sldMotor4.setMinimum(-3.1416)
      sldMotor4.setMaximum(3.1416)
      sldMotor4.setFocusPolicy(Qt.NoFocus)
      sldMotor4.setGeometry(30, 40, 100, 30)
      sldMotor4.doubleValueChanged[float].connect(self.changeValueMotor4)

      sldMotor5 = DoubleSlider(decimals=3)
      sldMotor5.setMinimum(-3.1416)
      sldMotor5.setMaximum(3.1416)
      sldMotor5.setFocusPolicy(Qt.NoFocus)
      sldMotor5.setGeometry(30, 40, 100, 30)
      sldMotor5.doubleValueChanged[float].connect(self.changeValueMotor5)

      sldMotor6 = DoubleSlider(decimals=3)
      sldMotor6.setMinimum(-3.1416)
      sldMotor6.setMaximum(3.1416)
      sldMotor6.setFocusPolicy(Qt.NoFocus)
      sldMotor6.setGeometry(30, 40, 100, 30)
      sldMotor6.doubleValueChanged[float].connect(self.changeValueMotor6)

      grid.addWidget(labelMotor1, 0, 0)
      grid.addWidget(sldMotor1, 0, 1)
      grid.addWidget(self.labelMotor1_value, 0, 2)

      grid.addWidget(labelMotor2, 1, 0)
      grid.addWidget(sldMotor2, 1, 1)
      grid.addWidget(self.labelMotor2_value, 1, 2)

      grid.addWidget(labelMotor3, 2, 0)
      grid.addWidget(sldMotor3, 2, 1)
      grid.addWidget(self.labelMotor3_value, 2, 2)

      grid.addWidget(labelMotor4, 3, 0)
      grid.addWidget(sldMotor4, 3, 1)
      grid.addWidget(self.labelMotor4_value, 3, 2)

      grid.addWidget(labelMotor5, 4, 0)
      grid.addWidget(sldMotor5, 4, 1)
      grid.addWidget(self.labelMotor5_value, 4, 2)

      grid.addWidget(labelMotor6, 5, 0)
      grid.addWidget(sldMotor6, 5, 1)
      grid.addWidget(self.labelMotor6_value, 5, 2)

      self.setWindowTitle("MARA joint states publisher")
      self.show()
      self.motor1 = 0.0
      self.motor2 = 0.0
      self.motor3 = 0.0
      self.motor4 = 0.0
      self.motor5 = 0.0
      self.motor6 = 0.0

    def changeValueMotor1(self, value):
        self.motor1 = float(value)
        self.sendJointStates()
        self.labelMotor1_value.setText(str(self.motor1))

    def changeValueMotor2(self, value):
        self.motor2 = float(value)
        self.sendJointStates()
        self.labelMotor2_value.setText(str(self.motor2))

    def changeValueMotor3(self, value):
        self.motor3 = float(value)
        self.sendJointStates()
        self.labelMotor3_value.setText(str(self.motor3))

    def changeValueMotor4(self, value):
        self.motor4 = float(value)
        self.sendJointStates()
        self.labelMotor4_value.setText(str(self.motor4))

    def changeValueMotor5(self, value):
        self.motor5 = float(value)
        self.sendJointStates()
        self.labelMotor5_value.setText(str(self.motor5))

    def changeValueMotor6(self, value):
        self.motor6 = float(value)
        self.sendJointStates()
        self.labelMotor6_value.setText(str(self.motor6))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
