#!/usr/bin/python3
# -*- coding: utf-8 -*-

#Qt5
from PyQt5.QtWidgets import (QWidget, QSlider, QGridLayout, QPushButton,
    QLabel, QApplication)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
#system
import sys
import time

#ROS 2.0
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from hrim_sensor_camera_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

import rclpy

import cv2
import numpy as np
import os


class Example(QWidget):

    def chatter_callback(self, msg):
        imagen_numpy = np.asarray(msg.data, dtype=np.uint8)
        img = cv2.imdecode(imagen_numpy, 1)
        cv2.imshow("Image window", img)
        cv2.waitKey(3)
        print('chatter_callback')

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):

        rclpy.init(args=None)

        self.node = rclpy.create_node('test_finger_control_service')
        self.publisher = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_B827EB79FCFB/Command')
        self.publisher_axis2 = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_B827EBCB8804/Command')

        self.publisher_pal = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_B827EBFA06A8/Command')

        grid = QGridLayout()
        self.setLayout(grid)

        self.labelPosition = QLabel("Position")
        sldPosition = QSlider(Qt.Horizontal, self)
        sldPosition.setMinimum(-90)
        sldPosition.setMaximum(90)
        sldPosition.setFocusPolicy(Qt.NoFocus)
        sldPosition.setGeometry(30, 40, 100, 30)
        sldPosition.setTickInterval(0.1)
        sldPosition.valueChanged[int].connect(self.changeValuePosition)

        self.labelPosition_axis2 = QLabel("Position")
        sldPosition_axis2 = QSlider(Qt.Horizontal, self)
        sldPosition_axis2.setMinimum(-90)
        sldPosition_axis2.setMaximum(90)
        sldPosition_axis2.setFocusPolicy(Qt.NoFocus)
        sldPosition_axis2.setGeometry(30, 40, 100, 30)
        sldPosition_axis2.setTickInterval(0.1)
        sldPosition_axis2.valueChanged[int].connect(self.changeValuePosition_axis2)

        self.labelPosition_pal = QLabel("Position")
        sldPosition_pal = QSlider(Qt.Horizontal, self)
        sldPosition_pal.setMaximum(90)
        sldPosition_pal.setMinimum(-90)
        sldPosition_pal.setFocusPolicy(Qt.NoFocus)
        sldPosition_pal.setGeometry(30, 40, 100, 30)
        sldPosition_pal.setTickInterval(0.1)
        sldPosition_pal.valueChanged[int].connect(self.changeValuePosition_pal)

        grid.addWidget(self.labelPosition, 0, 0)
        grid.addWidget(sldPosition, 0, 1)
        grid.addWidget(self.labelPosition_axis2, 1, 0)
        grid.addWidget(sldPosition_axis2, 1, 1)
        grid.addWidget(self.labelPosition_pal, 2, 0)
        grid.addWidget(sldPosition_pal, 2, 1)

        self.setWindowTitle("HANS motor")
        self.show()
        self.pos = 0.0
        self.vel = 30.0
        self.effort = 10.0

        self.value_hans_axis_1 = 0;
        self.value_hans_axis_2 = 0;
        self.value_pal = 0;

        subscription = self.node.create_subscription(CompressedImage,
                                                    '/hrim_sensing_camera_B827EBF38D04/compressed_image',
                                                     self.chatter_callback,
                                                     qos_profile=qos_profile_sensor_data)

        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(50)

    def update(self):
        # rclpy.spin_once(self.node)
        msg = GoalRotaryServo()
        msg.position = self.value_pal * 3.1416/180
        msg.control_type = 1
        msg.velocity = 0.1
        self.publisher_pal.publish(msg)

        self.labelPosition_pal.setText("Position " + str(self.value_pal))
        msg = GoalRotaryServo()
        msg.position = self.value_hans_axis_2 * 3.1416/180
        self.publisher_axis2.publish(msg)
        self.labelPosition_axis2.setText("Position " + str(self.value_hans_axis_2))

        msg = GoalRotaryServo()
        msg.position = self.value_hans_axis_1 * 3.1416/180
        self.publisher.publish(msg)
        self.labelPosition.setText("Position " + str(self.value_hans_axis_1))

    def changeValuePosition_pal(self, value):
        self.value_pal = value
        msg = GoalRotaryServo()
        msg.position = self.value_pal * 3.1416/180
        msg.velocity = 0.01
        msg.control_type = 4
        self.publisher_pal.publish(msg)
        self.labelPosition_pal.setText("Position " + str(value))

    def changeValuePosition_axis2(self, value):
        self.value_hans_axis_2 = value
        msg = GoalRotaryServo()
        msg.position = self.value_hans_axis_2 * 3.1416/180
        self.publisher_axis2.publish(msg)
        self.labelPosition_axis2.setText("Position " + str(value))

    def changeValuePosition(self, value):
        self.value_hans_axis_1 = value
        msg = GoalRotaryServo()
        msg.position = self.value_hans_axis_1 * 3.1416/180
        self.publisher.publish(msg)
        self.labelPosition.setText("Position " + str(value))

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
