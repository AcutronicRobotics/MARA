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
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

import rclpy

import numpy as np
import os
beloop= False

class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):

        rclpy.init(args=None)

        self.node = rclpy.create_node('test_finger_control_service')

        self.publisher = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_0000000000012/goal', qos_profile=qos_profile_sensor_data)
        self.publisher_axis2 = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_000000000001/goal', qos_profile=qos_profile_sensor_data)

        self.publisher_pal = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_000000000002/goal', qos_profile=qos_profile_sensor_data)
        self.publisher_hebi = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_0000000000022/goal', qos_profile=qos_profile_sensor_data)

        self.publisher_lander = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_000000000003/goal', qos_profile=qos_profile_sensor_data)
        self.publisher_lander2 = self.node.create_publisher(GoalRotaryServo, '/hrim_actuation_servomotor_0000000000032/goal', qos_profile=qos_profile_sensor_data)

        grid = QGridLayout()
        self.setLayout(grid)

        self.labelPosition = QLabel("Position")
        self.sldPosition = QSlider(Qt.Horizontal, self)
        self.sldPosition.setMinimum(-180)
        self.sldPosition.setMaximum(180)
        self.sldPosition.setFocusPolicy(Qt.NoFocus)
        self.sldPosition.setGeometry(30, 40, 100, 30)
        self.sldPosition.setTickInterval(0.1)
        self.sldPosition.valueChanged[int].connect(self.changeValuePosition)

        self.labelPosition_axis2 = QLabel("Position")
        self.sldPosition_axis2 = QSlider(Qt.Horizontal, self)
        self.sldPosition_axis2.setMinimum(-180)
        self.sldPosition_axis2.setMaximum(180)
        self.sldPosition_axis2.setFocusPolicy(Qt.NoFocus)
        self.sldPosition_axis2.setGeometry(30, 40, 100, 30)
        self.sldPosition_axis2.setTickInterval(0.1)
        self.sldPosition_axis2.valueChanged[int].connect(self.changeValuePosition_axis2)

        self.labelPosition_pal = QLabel("Position")
        self.sldPosition_pal = QSlider(Qt.Horizontal, self)
        self.sldPosition_pal.setMaximum(180)
        self.sldPosition_pal.setMinimum(-180)
        self.sldPosition_pal.setFocusPolicy(Qt.NoFocus)
        self.sldPosition_pal.setGeometry(30, 40, 100, 30)
        self.sldPosition_pal.setTickInterval(0.1)
        self.sldPosition_pal.valueChanged[int].connect(self.changeValuePosition_pal)

        self.labelPosition_hebi = QLabel("Position")
        self.sldPosition_hebi = QSlider(Qt.Horizontal, self)
        self.sldPosition_hebi.setMaximum(180)
        self.sldPosition_hebi.setMinimum(-180)
        self.sldPosition_hebi.setFocusPolicy(Qt.NoFocus)
        self.sldPosition_hebi.setGeometry(30, 40, 100, 30)
        self.sldPosition_hebi.setTickInterval(0.1)
        self.sldPosition_hebi.valueChanged[int].connect(self.changeValuePosition_hebi)

        self.labelPosition_lander = QLabel("Position")
        self.sldPosition_lander = QSlider(Qt.Horizontal, self)
        self.sldPosition_lander.setMaximum(180)
        self.sldPosition_lander.setMinimum(-180)
        self.sldPosition_lander.setFocusPolicy(Qt.NoFocus)
        self.sldPosition_lander.setGeometry(30, 40, 100, 30)
        self.sldPosition_lander.setTickInterval(0.1)
        self.sldPosition_lander.valueChanged[int].connect(self.changeValuePosition_lander)

        self.labelPosition_lander2 = QLabel("Position")
        self.sldPosition_lander2 = QSlider(Qt.Horizontal, self)
        self.sldPosition_lander2.setMaximum(180)
        self.sldPosition_lander2.setMinimum(-180)
        self.sldPosition_lander2.setFocusPolicy(Qt.NoFocus)
        self.sldPosition_lander2.setGeometry(30, 40, 100, 30)
        self.sldPosition_lander2.setTickInterval(0.1)
        self.sldPosition_lander2.valueChanged[int].connect(self.changeValuePosition_lander2)


        grid.addWidget(self.labelPosition, 0, 0)
        grid.addWidget(self.sldPosition, 0, 1)
        grid.addWidget(self.labelPosition_axis2, 1, 0)
        grid.addWidget(self.sldPosition_axis2, 1, 1)
        grid.addWidget(self.labelPosition_pal, 2, 0)
        grid.addWidget(self.sldPosition_pal, 2, 1)
        grid.addWidget(self.labelPosition_hebi, 3, 0)
        grid.addWidget(self.sldPosition_hebi, 3, 1)
        grid.addWidget(self.labelPosition_lander, 4, 0)
        grid.addWidget(self.sldPosition_lander, 4, 1)
        grid.addWidget(self.labelPosition_lander2, 5, 0)
        grid.addWidget(self.sldPosition_lander2, 5, 1)


        self.button = QPushButton('Go 0', self)
        self.button.clicked.connect(self.gotozero)
        self.button2 = QPushButton('Go A', self)
        self.button2.clicked.connect(self.gotoa)
        self.button3 = QPushButton('Go B', self)
        self.button3.clicked.connect(self.gotob)
        self.button4 = QPushButton('LOOP', self)
        self.button4.clicked.connect(self.goloop)
        self.button5 = QPushButton('STOP LOOP', self)
        self.button5.clicked.connect(self.stoploop)

        grid.addWidget(self.button, 6, 0)
        grid.addWidget(self.button2, 7, 0)
        grid.addWidget(self.button3,8, 0)
        grid.addWidget(self.button4,9, 0)
        grid.addWidget(self.button5,9, 1)

        self.setWindowTitle("MAIRA")
        self.show()
        self.pos = 0.0
        self.vel = 30.0
        self.effort = 10.0

        self.value_hans_axis_1 = 0;
        self.value_hans_axis_2 = 0;
        self.value_pal = 0;
        self.value_hebi = 0;
        self.value_lander = 0;
        self.value_lander2 = 0;

    def update(self):
        rclpy.spin_once(self.node)

    def changeValuePosition_hebi(self, value):
        self.value_hebi = value
        msg = GoalRotaryServo()
        msg.position = self.value_hebi * 3.1416/180
        msg.velocity = 0.404
        msg.control_type = 1
        self.publisher_hebi.publish(msg)
        self.labelPosition_hebi.setText("Position " + str(self.value_hebi))


    def changeValuePosition_pal(self, value):
        self.value_pal = value
        msg = GoalRotaryServo()
        msg.position = self.value_pal * 3.1416/180
        msg.velocity = 0.404
        msg.control_type = 1
        self.publisher_pal.publish(msg)
        self.labelPosition_pal.setText("Position " + str(value))

    def changeValuePosition_axis2(self, value):
        self.value_hans_axis_2 = value
        msg = GoalRotaryServo()
        msg.position = self.value_hans_axis_2 * 3.1416/180
        msg.velocity = 0.404
        msg.control_type = 1
        self.publisher_axis2.publish(msg)
        self.labelPosition_axis2.setText("Position " + str(value))

    def changeValuePosition(self, value):
        self.value_hans_axis_1 = value
        msg = GoalRotaryServo()
        msg.velocity = 0.404
        msg.control_type = 1
        msg.position = self.value_hans_axis_1 * 3.1416/180
        self.publisher.publish(msg)
        self.labelPosition.setText("Position " + str(value))

    def changeValuePosition_lander(self, value):

        self.value_lander = value
        msg = GoalRotaryServo()
        msg.velocity = 0.404
        msg.position = self.value_lander * 3.1416/180
        self.publisher_lander.publish(msg)
        self.labelPosition_lander.setText("Position " + str(value))

    def changeValuePosition_lander2(self, value):
        self.value_hans_axis_1 = value
        msg = GoalRotaryServo()
        msg.velocity = 0.404
        msg.position = self.value_hans_axis_1 * 3.1416/180
        self.publisher_lander2.publish(msg)
        self.labelPosition_lander2.setText("Position " + str(value))

    def gotozero(self):
        print("gotozero")
        self.sldPosition.setValue(0)
        self.sldPosition_axis2.setValue(0)
        self.sldPosition_pal.setValue(0)
        self.sldPosition_hebi.setValue(0)
        self.sldPosition_lander.setValue(0)
        self.sldPosition_lander2.setValue(0)
    def gotoa(self):
        print("gotozero")
        self.sldPosition.setValue(40)
        self.sldPosition_axis2.setValue(40)
        self.sldPosition_pal.setValue(40)
        self.sldPosition_hebi.setValue(40)
        self.sldPosition_lander.setValue(40)
        self.sldPosition_lander2.setValue(40)
    def gotob(self):
        print("gotozero")
        self.sldPosition.setValue(-61)
        self.sldPosition_axis2.setValue(-60)
        self.sldPosition_pal.setValue(-39)
        self.sldPosition_hebi.setValue(-63)
        self.sldPosition_lander.setValue(21)
        self.sldPosition_lander2.setValue(-36)
    def goloop(self):
        global beloop
        beloop = True
        while beloop:
            print("go Loop 40")
            self.sldPosition.setValue(40)
            self.sldPosition_axis2.setValue(40)
            self.sldPosition_pal.setValue(40)
            self.sldPosition_hebi.setValue(40)
            self.sldPosition_lander.setValue(40)
            self.sldPosition_lander2.setValue(40)

            time.sleep(14)
            print("go Loop 0")
            self.sldPosition.setValue(-40)
            self.sldPosition_axis2.setValue(-40)
            self.sldPosition_pal.setValue(-40)
            self.sldPosition_hebi.setValue(-40)
            self.sldPosition_lander.setValue(-40)
            self.sldPosition_lander2.setValue(-40)
            time.sleep(14)

    def stoploop(self):
        global beloop
        beloop = False
if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
