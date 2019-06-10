
#include <iostream>
#include <memory>
#include <string>

// include ROS 1
#include "ros/ros.h"
#include "ros/message.h"
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "geometry_msgs/PoseStamped.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


ros::Publisher ros1_pub;

void ros2Callback(const geometry_msgs::msg::PoseStamped::SharedPtr ros2_msg)
{
  printf("Received message from ROS2!\n");

  geometry_msgs::PoseStamped ros1_msg;
  ros1_msg.header.stamp = ros::Time::now();
  ros1_msg.header.frame_id = ros2_msg->header.frame_id;
  ros1_msg.pose.position.y = ros2_msg->pose.position.y;
  ros1_msg.pose.position.x = ros2_msg->pose.position.x;
  ros1_msg.pose.position.z = ros2_msg->pose.position.z;
  ros1_msg.pose.orientation.w = ros2_msg->pose.orientation.w;
  ros1_msg.pose.orientation.x = ros2_msg->pose.orientation.x;
  ros1_msg.pose.orientation.y = ros2_msg->pose.orientation.y;
  ros1_msg.pose.orientation.z = ros2_msg->pose.orientation.z;

  printf("Passing along to ROS 1\n");
  ros1_pub.publish(ros1_msg);
}


rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros2_pub;

// void ros1Callback(const ros::MessageEvent<std_msgs::String const> & ros1_msg_event)
// {
//   const boost::shared_ptr<ros::M_string> & connection_header =
//     ros1_msg_event.getConnectionHeaderPtr();
//   std::string key = "callerid";
//   if (connection_header->find(key) != connection_header->end()) {
//     if (connection_header->at(key) == "/ros_bridge") {
//       printf("    I heard from ROS 1 from myself\n");
//       return;
//     }
//     printf("I heard from ROS 1 from: [%s]\n", connection_header->at(key).c_str());
//   }
//
//   const boost::shared_ptr<std_msgs::String const> & ros1_msg = ros1_msg_event.getConstMessage();
//   printf("I heard from ROS 1: [%s]\n", ros1_msg->data.c_str());
//
//   std_msgs::msg::String ros2_msg;
//   ros2_msg.data = ros1_msg->data;
//   printf("Passing along to ROS 2: [%s]\n", ros2_msg.data.c_str());
//   ros2_pub->publish(ros2_msg);
// }


int main(int argc, char * argv[])
{

  // ROS 1 node and publisher
  ros::init(argc, argv, "cable_bridge_ros1");
  ros::NodeHandle ros1_node;
  ros1_pub = ros1_node.advertise<geometry_msgs::PoseStamped>("/mara/pred_target", 10);
  // ros::Subscriber ros1_sub = ros1_node.subscribe(
  //   "/mara/pred_target", 10, ros1Callback);

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("cable_bridge_ros2");
  // auto pub_ros2 = ros2_node->create_publisher<geometry_msgs::msg::Pose>("/mara/pred_target");
  auto sub_ros2 = ros2_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mara/pred_target", ros2Callback);

  //////////////////////////////////////////////////////////////////////////
  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::MultiThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(100));
  }

  return 0;
}
