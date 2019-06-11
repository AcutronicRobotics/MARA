#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import yaml
import rclpy
import argparse

from hrim_actuator_gripper_srvs.srv import ControlFinger
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QSlider, QGridLayout, QPushButton, QLabel, QApplication

class Example(QWidget):

    def __init__(self, goalTopics, gripper, gripperService):
        super().__init__()

        rclpy.init(args=None)
        self.node = rclpy.create_node('mara_control')

        self.pubs = []
        for gt in goalTopics:
            self.createPublisher(gt)

        self.initUI(gripper, gripperService)

    def createPublisher(self, topicName):
        pub = self.node.create_publisher(GoalRotaryServo, topicName, qos_profile=qos_profile_sensor_data)
        self.pubs.append(pub)

    def initUI(self, gripper, gripperService):
        self.gripper = gripper

        self.msg = GoalRotaryServo()
        self.msg.velocity = 0.1
        self.msg.control_type = 4

        grid = QGridLayout()
        self.setLayout(grid)

        if self.gripper != 'None':
            self.client = self.node.create_client(ControlFinger, gripperService)
            while not self.client.wait_for_service(timeout_sec=1.0):
                print(gripperService + ' service not available, waiting again...')

            self.req = ControlFinger.Request()

            if self.gripper == '140':
                self.req.goal_velocity = 30.0
            else:
                self.req.goal_velocity = 20.0

            self.labelGripperPos = QLabel("Gripper: 0")
            self.sldGripperPos = QSlider(Qt.Horizontal, self)
            self.sldGripperPos.setMinimum(0)
            self.sldGripperPos.setMaximum(87)
            self.sldGripperPos.setFocusPolicy(Qt.NoFocus)
            self.sldGripperPos.setGeometry(30, 40, 100, 30)
            self.sldGripperPos.setTickInterval(1)
            self.sldGripperPos.valueChanged[int].connect(self.changePositionGripper)

            self.labelGripperVel = QLabel("Speed: 30")
            self.sldGripperVel = QSlider(Qt.Horizontal, self)

            if self.gripper == '140':
                self.sldGripperVel.setMinimum(300)
                self.sldGripperVel.setMaximum(2500)
            else:
                self.sldGripperVel.setMinimum(1500)
                self.sldGripperVel.setMaximum(200)

            self.sldGripperVel.setFocusPolicy(Qt.NoFocus)
            self.sldGripperVel.setGeometry(30, 40, 100, 30)
            self.sldGripperVel.setTickInterval(1)
            self.sldGripperVel.valueChanged[int].connect(self.changeVelocityGripper)

            grid.addWidget(self.labelGripperPos, 10, 0)
            grid.addWidget(self.sldGripperPos, 10, 1)
            grid.addWidget(self.labelGripperVel, 11, 0)
            grid.addWidget(self.sldGripperVel, 11, 1)

        self.labelJoint1 = QLabel("Joint 1: 0")
        self.sldJoint1 = QSlider(Qt.Horizontal, self)
        self.sldJoint1.setMinimum(-180)
        self.sldJoint1.setMaximum(180)
        self.sldJoint1.setFocusPolicy(Qt.NoFocus)
        self.sldJoint1.setGeometry(30, 40, 100, 30)
        self.sldJoint1.setTickInterval(0.1)
        self.sldJoint1.valueChanged[int].connect(self.changePositionJoint1)

        self.labelJoint2 = QLabel("Joint 2: 0")
        self.sldJoint2 = QSlider(Qt.Horizontal, self)
        self.sldJoint2.setMinimum(-180)
        self.sldJoint2.setMaximum(180)
        self.sldJoint2.setFocusPolicy(Qt.NoFocus)
        self.sldJoint2.setGeometry(30, 40, 100, 30)
        self.sldJoint2.setTickInterval(0.1)
        self.sldJoint2.valueChanged[int].connect(self.changePositionJoint2)

        self.labelJoint3 = QLabel("Joint 3: 0")
        self.sldJoint3 = QSlider(Qt.Horizontal, self)
        self.sldJoint3.setMaximum(180)
        self.sldJoint3.setMinimum(-180)
        self.sldJoint3.setFocusPolicy(Qt.NoFocus)
        self.sldJoint3.setGeometry(30, 40, 100, 30)
        self.sldJoint3.setTickInterval(0.1)
        self.sldJoint3.valueChanged[int].connect(self.changePositionJoint3)

        self.labelJoint4 = QLabel("Joint 4: 0")
        self.sldJoint4 = QSlider(Qt.Horizontal, self)
        self.sldJoint4.setMinimum(-180)
        self.sldJoint4.setMaximum(180)
        self.sldJoint4.setFocusPolicy(Qt.NoFocus)
        self.sldJoint4.setGeometry(30, 40, 100, 30)
        self.sldJoint4.setTickInterval(0.1)
        self.sldJoint4.valueChanged[int].connect(self.changePositionJoint4)

        self.labelJoint5 = QLabel("Joint 5: 0")
        self.sldJoint5 = QSlider(Qt.Horizontal, self)
        self.sldJoint5.setMinimum(-180)
        self.sldJoint5.setMaximum(180)
        self.sldJoint5.setFocusPolicy(Qt.NoFocus)
        self.sldJoint5.setGeometry(30, 40, 100, 30)
        self.sldJoint5.setTickInterval(0.1)
        self.sldJoint5.valueChanged[int].connect(self.changePositionJoint5)

        self.labelJoint6 = QLabel("Joint 6: 0")
        self.sldJoint6 = QSlider(Qt.Horizontal, self)
        self.sldJoint6.setMinimum(-180)
        self.sldJoint6.setMaximum(180)
        self.sldJoint6.setFocusPolicy(Qt.NoFocus)
        self.sldJoint6.setGeometry(30, 40, 100, 30)
        self.sldJoint6.setTickInterval(0.1)
        self.sldJoint6.valueChanged[int].connect(self.changePositionJoint6)

        self.labelVelocity = QLabel("Speed: 0.3")
        self.sldVelocity = QSlider(Qt.Horizontal, self)
        self.sldVelocity.setMinimum(10)
        self.sldVelocity.setMaximum(157)
        self.sldVelocity.setFocusPolicy(Qt.NoFocus)
        self.sldVelocity.setGeometry(30, 40, 100, 30)
        self.sldVelocity.setTickInterval(1)
        self.sldVelocity.valueChanged[int].connect(self.changeVelocity)

        grid.addWidget(self.labelJoint1, 0, 0)
        grid.addWidget(self.sldJoint1, 0, 1)
        grid.addWidget(self.labelJoint2, 1, 0)
        grid.addWidget(self.sldJoint2, 1, 1)
        grid.addWidget(self.labelJoint3, 2, 0)
        grid.addWidget(self.sldJoint3, 2, 1)
        grid.addWidget(self.labelJoint4, 3, 0)
        grid.addWidget(self.sldJoint4, 3, 1)
        grid.addWidget(self.labelJoint5, 4, 0)
        grid.addWidget(self.sldJoint5, 4, 1)
        grid.addWidget(self.labelJoint6, 5, 0)
        grid.addWidget(self.sldJoint6, 5, 1)
        grid.addWidget(self.labelVelocity, 6, 0)
        grid.addWidget(self.sldVelocity, 6, 1)

        self.buttonHome = QPushButton('Go Home', self)
        self.buttonHome.clicked.connect(self.goHome)
        self.buttonA = QPushButton('Go A', self)
        self.buttonA.clicked.connect(self.goA)
        self.buttonB = QPushButton('Go B', self)
        self.buttonB.clicked.connect(self.goB)

        grid.addWidget(self.buttonHome, 7, 0)
        grid.addWidget(self.buttonA, 8, 0)
        grid.addWidget(self.buttonB, 8, 1)

        self.setWindowTitle("MARA")
        self.show()

    def update(self):
        rclpy.spin_once(self.node)

    def changePositionJoint1(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[0].publish(self.msg)
        self.labelJoint1.setText("Joint 1: " + str(value))

    def changePositionJoint2(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[1].publish(self.msg)
        self.labelJoint2.setText("Joint 2: " + str(value))

    def changePositionJoint3(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[2].publish(self.msg)
        self.labelJoint3.setText("Joint 3: " + str(value))

    def changePositionJoint4(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[3].publish(self.msg)
        self.labelJoint4.setText("Joint 4: " + str(value))

    def changePositionJoint5(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[4].publish(self.msg)
        self.labelJoint5.setText("Joint 5: " + str(value))

    def changePositionJoint6(self, value):
        self.msg.position = value * 3.1416/180
        self.pubs[5].publish(self.msg)
        self.labelJoint6.setText("Joint 6: " + str(value))

    def changeVelocity(self, value):
        self.msg.velocity = value/100
        self.labelVelocity.setText("Speed: " + str(value/100))

    def goHome(self):
        self.sldJoint1.setValue(0)
        self.sldJoint2.setValue(0)
        self.sldJoint3.setValue(0)
        self.sldJoint4.setValue(0)
        self.sldJoint5.setValue(0)
        self.sldJoint6.setValue(0)

    def goA(self):
        self.sldJoint1.setValue(40)
        self.sldJoint2.setValue(40)
        self.sldJoint3.setValue(40)
        self.sldJoint4.setValue(40)
        self.sldJoint5.setValue(40)
        self.sldJoint6.setValue(40)

    def goB(self):
        self.sldJoint1.setValue(-61)
        self.sldJoint2.setValue(-60)
        self.sldJoint3.setValue(-39)
        self.sldJoint4.setValue(-63)
        self.sldJoint5.setValue(21)
        self.sldJoint6.setValue(-36)

    def changePositionGripper(self, value):
        if self.gripper == 'hande':
            self.req.goal_linearposition = value/100

        else:
            self.req.goal_angularposition = value/100

        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())
        else:
            self.labelGripperPos.setText("Gripper: " + str(value/100))

    def changeVelocityGripper(self, value):
        self.req.goal_velocity = value/10

        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())
        else:
            self.labelGripperVel.setText("Speed: " + str(value/10))

class YAML():
    def __init__(self, yamlpath, gripper):
        self.yamlFile = open(yamlpath, 'r')
        self.gripper = gripper
        self.goalTopics = []
        self.goalGripperService = None

    def getTopics(self, yamlFile, env, rdi):
        try:
            y = yaml.safe_load(yamlFile)
            if env == "sim":
                motors = y['simulated_motors']
                if self.gripper != 'None':
                    self.goalGripperService = y['simulated_grippers'][1]
            elif env == "real":
                motors = y['real_motors']
                if self.gripper != 'None':
                    self.goalGripperService = y['real_grippers'][1]

                os.environ["ROS_DOMAIN_ID"] = rdi
                os.environ["RMW_IMPLEMENTATION"] = "rmw_opensplice_cpp"

            delimiter = "trajectory"
            for m in motors:
                name = m.split(delimiter)[0]
                axis = m.split(delimiter)[1]
                goaltopic = name + "goal" + axis
                self.goalTopics.append(goaltopic)

        except yaml.YAMLError as exc:
            print(exc)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='MARA simulated or real')
    parser.add_argument('-env', '--environment', type=str , choices=['sim', 'real'], default='sim', help="simulated or real")
    parser.add_argument('-id', '--rdi', type=str, default='22', help="ROS_DOMAIN_ID of the SoM")
    parser.add_argument('-g', '--gripper', type=str, choices=['None', '140', '85', 'hande'], default='None', help="MARA robot gripper")
    args = parser.parse_args()

    yamlpath = os.path.join(get_package_share_directory('hros_cognition_mara_components'), 'motors.yaml')
    yamlOBJ = YAML(yamlpath, args.gripper)
    yamltopics = yamlOBJ.getTopics(yamlOBJ.yamlFile, args.environment, args.rdi)

    app = QApplication(sys.argv)
    ex = Example(yamlOBJ.goalTopics, args.gripper, yamlOBJ.goalGripperService)
    sys.exit(app.exec_())
