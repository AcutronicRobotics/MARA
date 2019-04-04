#!/usr/bin/python3
# -*- coding: utf-8 -*-

#Qt5
from PyQt5.QtWidgets import (QWidget, QSlider, QGridLayout, QPushButton,
    QLabel, QApplication)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

#system
import sys
import time

#ROS 2.0
from hrim_actuator_gripper_srvs.srv import ControlFinger
import rclpy

class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def moveGripper(self):
        print('moveGripper', self.vel , self.pos, self.effort)
        req = ControlFinger.Request()
        req.goal_velocity = self.vel         # velocity range: 30 -  250 mm/s
        req.goal_effort = self.effort        # forces range:   10 -  125 N
        req.goal_linearposition = self.pos  # position range   0 - 0.87 rad

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Goal accepted: %d: ' % future.result().goal_accepted)
        else:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())

        # print('Goal accepted: %d' % self.cli.response.goal_accepted)

    def initUI(self):

        rclpy.init(args=None)
        self.node = rclpy.create_node('test_finger_control_service')

        self.cli = self.node.create_client(ControlFinger, '/hrim_actuation_gripper_000000000004/goal') #Change me!

        grid = QGridLayout()
        self.setLayout(grid)

        labelSpeed = QLabel("Speed")
        labelEffort = QLabel("Effort")
        labelPosition = QLabel("Position")

        button = QPushButton("Goto")

        sldSpeed = QSlider(Qt.Horizontal, self)
        sldSpeed.setMinimum(30)
        sldSpeed.setMaximum(250)
        sldSpeed.setFocusPolicy(Qt.NoFocus)
        sldSpeed.valueChanged[int].connect(self.changeValueSpeed)

        sldPosition = QSlider(Qt.Horizontal, self)
        sldPosition.setMinimum(0)
        sldPosition.setMaximum(86)
        sldPosition.setFocusPolicy(Qt.NoFocus)
        sldPosition.setGeometry(30, 40, 100, 30)
        sldPosition.valueChanged[int].connect(self.changeValuePosition)

        sldEffort = QSlider(Qt.Horizontal, self)
        sldEffort.setMinimum(10)
        sldEffort.setMaximum(125)
        sldEffort.setFocusPolicy(Qt.NoFocus)
        sldEffort.valueChanged[int].connect(self.changeValueEffort)

        grid.addWidget(labelPosition, 0, 0)
        grid.addWidget(sldPosition, 0, 1)

        grid.addWidget(labelSpeed, 1, 0)
        grid.addWidget(sldSpeed, 1, 1)

        grid.addWidget(labelEffort, 2, 0)
        grid.addWidget(sldEffort, 2, 1)

        grid.addWidget(button, 3, 0, 1, 2)
        button.clicked.connect(self.clickedOnMoveGripper)

        self.setWindowTitle("Robotiq 140 interface")
        self.show()
        self.pos = 0.0
        self.vel = 30.0
        self.effort = 10.0


    def clickedOnMoveGripper(self):
        self.moveGripper()

    def changeValueSpeed(self, value):
        self.vel = float(value)

    def changeValuePosition(self, value):
        self.pos = float(value/100.0)

    def changeValueEffort(self, value):
        self.vel = float(value)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
