/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo_msgs/msg/contact_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <gazebo_ros/node.hpp>
#include <gazebo/transport/Node.hh>

#include <gazebo/gazebo_client.hh>

#include <iostream>

gazebo_ros::Node::SharedPtr ros_node;

rclcpp::Publisher<gazebo_msgs::msg::ContactState>::SharedPtr contacts_pub ;

gazebo::transport::NodePtr gz_node;

// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
  gazebo_msgs::msg::ContactState contact;
  geometry_msgs::msg::Vector3 position;
  geometry_msgs::msg::Vector3 normals;


  if (_msg->contact_size() > 0){
    contact.collision1_name = _msg->contact(0).collision1();
    contact.collision2_name = _msg->contact(0).collision2();

    for (int j = 0; j <  _msg->contact(0).position_size(); ++j){
      contact.collision1_name = _msg->contact(0).collision1();
      contact.collision2_name = _msg->contact(0).collision2();

      position.x =  _msg->contact(0).position(j).x();
      position.y =  _msg->contact(0).position(j).y();
      position.z =  _msg->contact(0).position(j).z();
      contact.contact_positions.push_back(position);

      normals.x =  _msg->contact(0).normal(j).x();
      normals.y =  _msg->contact(0).normal(j).y();
      normals.z =  _msg->contact(0).normal(j).z();
      contact.contact_normals.push_back(normals);

      contact.depths.push_back(static_cast<float>(_msg->contact(0).depth(0)));
    }
  }
  contacts_pub->publish(contact);

}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  // Load gazebo
  gazebo::client::setup(argc, argv);

  if (!rclcpp::is_initialized()) {
    rclcpp::init(argc, argv);
    ros_node = gazebo_ros::Node::Get();
  } else {
    ros_node = gazebo_ros::Node::Get();
  }

  contacts_pub = ros_node->create_publisher<gazebo_msgs::msg::ContactState>("/gazebo_contacts", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)));

  // Gazebo transport
  gz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = gz_node->Subscribe("~/physics/contacts", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
