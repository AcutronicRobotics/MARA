#!/usr/bin/python3
# -*- coding: utf-8 -*-

from gazebo_msgs.srv import SpawnEntity

import rclpy
from ros2pkg.api import get_prefix_path
import os

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    prefix_path = get_prefix_path('robotiq_hande_gripper_description')
    assert os.path.isdir(prefix_path)

    content = ""
    with open(prefix_path + "/share/robotiq_hande_gripper_description/urdf/robotiq_hande.urdf", 'r') as content_file:
        content = content_file.read()

    req = SpawnEntity.Request()
    req.name = "hande"
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
