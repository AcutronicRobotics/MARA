#include <iostream>
#include <yaml-cpp/yaml.h>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "sensor_msgs/JointState.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rcutils/cmdline_parser.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unistd.h>
#include <cstring>

ros::Publisher pub_joint_state_ros1;
std::vector<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> list_pub_trajectory;
std::vector<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> pub_ros2_lista;

void motorStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr ros2_msg)
{

  sensor_msgs::JointState ros1_joint_state_msg;
  ros1_joint_state_msg.header.stamp = ros::Time::now();

  ros1_joint_state_msg.name.resize(ros2_msg->joint_names.size());
  for(unsigned int i = 0; i < ros2_msg->joint_names.size(); i++){
    ros1_joint_state_msg.name[i] = "motor" + std::to_string(i+1);//ros2_msg->joint_names[i];
  }

  ros1_joint_state_msg.position.resize(ros2_msg->actual.positions.size());
  for(unsigned int j = 0; j < ros1_joint_state_msg.position.size(); j++){
    ros1_joint_state_msg.position[j] = ros2_msg->actual.positions[j];
  }

  pub_joint_state_ros1.publish(ros1_joint_state_msg);
}

int main(int argc, char * argv[])
{

  char hostname[150];
  memset(hostname, 0, 150);
  if(gethostname(hostname, 150)==-1){
    return -2;
  }

  std::string node_name = std::string("bridge");
  std::replace(node_name.begin(), node_name.end(), '-', '_');

  // ROS 1 node and publisher
  ros::init(argc, argv, "mara_bridge");
  ros::NodeHandle ros1_node;

  std::string file_motors;
  if (rcutils_cli_option_exist(argv, argv + argc, "-motors")){
    file_motors = std::string(rcutils_cli_get_option(argv, argv + argc, "-motors"));
  }

  std::cout << "Trying to open  " << file_motors << std::endl;

  YAML::Node config = YAML::LoadFile(file_motors);
  if(config.IsNull()){
      std::cout << "return";
      return -1;
  }
  std::cout << config.size() << std::endl;

  std::vector<std::string> lista_subcribers;

  for (auto motor : config["motors"]) {
    std::string s = motor.as<std::string>();
    lista_subcribers.push_back(s);
  }

  pub_ros2_lista.resize(lista_subcribers.size());
  for (unsigned int j = 0; j < lista_subcribers.size(); j++){
    pub_ros2_lista[j] = NULL;
  }

  std::vector<ros::Subscriber> lista_subcribers_ros1;

  for(unsigned int i = 0; i < lista_subcribers.size(); i++){

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_ros2 = pub_ros2_lista[i];

    std::cout << "Creating ROS 1 subscriber " << lista_subcribers[i] << "!" << std::endl;

    boost::function<void (const trajectory_msgs::JointTrajectory&)> callback =
    [i] (const trajectory_msgs::JointTrajectory& ros1_msg) {
      auto ros2_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();

      ros2_msg->points.resize(ros1_msg.points.size());
      for (unsigned int i = 0; i < ros1_msg.points.size(); i++){
        ros2_msg->points[i].positions.resize(ros1_msg.points[i].positions.size());
        for (unsigned int j = 0; j < ros1_msg.points[i].positions.size(); j++ )
          ros2_msg->points[i].positions[j] = ros1_msg.points[i].positions[j];

        ros2_msg->points[i].velocities.resize(ros1_msg.points[i].velocities.size());
        for (unsigned int j = 0; j < ros1_msg.points[i].velocities.size(); j++ )
          ros2_msg->points[i].velocities[j] = ros1_msg.points[i].velocities[j];

        ros2_msg->points[i].accelerations.resize(ros1_msg.points[i].accelerations.size());
        for (unsigned int j = 0; j < ros1_msg.points[i].accelerations.size(); j++ )
          ros2_msg->points[i].accelerations[j] = ros1_msg.points[i].accelerations[j];

        ros2_msg->points[i].time_from_start.sec = ros1_msg.points[i].time_from_start.sec;
        ros2_msg->points[i].time_from_start.nanosec = ros1_msg.points[i].time_from_start.nsec;
      }
      if(pub_ros2_lista[i]!=NULL)
        pub_ros2_lista[i]->publish(ros2_msg);
        // std::cout << "publish!! " << i << pub_ros2_lista[i]->get_topic_name() << std::endl;
    };
    lista_subcribers_ros1.push_back(ros1_node.subscribe<trajectory_msgs::msg::JointTrajectory>(lista_subcribers[i].c_str(), 1, callback));
  }
  pub_joint_state_ros1 = ros1_node.advertise<sensor_msgs::JointState>("/joint_states", 10);

  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("mara_bridge");

  for(unsigned int i = 0; i < lista_subcribers.size(); i++){
    std::cout << "Creating ROS 2 publisher " << lista_subcribers[i] << "!" << std::endl;

    pub_ros2_lista[i] =
              ros2_node->create_publisher<trajectory_msgs::msg::JointTrajectory>
                     (lista_subcribers[i], rmw_qos_profile_sensor_data);
  }

  auto sub_servoMotorGoal = ros2_node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/mara_controller/stateMotor", motorStateCallback, rmw_qos_profile_sensor_data);

  //////////////////////////////////////////////////////////////////////////7
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
