#include "FollowJointTrajectoryAction.hpp"

ros::Publisher pub_joint_state_ros1;
std::vector<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> list_pub_trajectory;
std::vector<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> pub_ros2_lista;
std::string environment;
std::string motor_key;

void motorStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr ros2_msg)
{
  sensor_msgs::JointState ros1_joint_state_msg;
  ros1_joint_state_msg.header.stamp = ros::Time::now();

  ros1_joint_state_msg.name.resize(ros2_msg->joint_names.size());
  for(unsigned int i = 0; i < ros2_msg->joint_names.size(); i++){
    ros1_joint_state_msg.name[i] = ros2_msg->joint_names[i];
  }

  ros1_joint_state_msg.position.resize(ros2_msg->actual.positions.size());
  for(unsigned int j = 0; j < ros1_joint_state_msg.position.size(); j++){
    ros1_joint_state_msg.position[j] = ros2_msg->actual.positions[j];
  }

  ros1_joint_state_msg.velocity.resize(ros2_msg->actual.velocities.size());
  for(unsigned int j = 0; j < ros1_joint_state_msg.velocity.size(); j++){
    ros1_joint_state_msg.velocity[j] = ros2_msg->actual.velocities[j];
  }

  ros1_joint_state_msg.effort.resize(ros2_msg->actual.effort.size());
  for(unsigned int j = 0; j < ros1_joint_state_msg.effort.size(); j++){
    ros1_joint_state_msg.effort[j] = ros2_msg->actual.effort[j];
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

  std::string file_motors, ft_topic;
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

  std::vector<std::string> lista_subscribers;

  environment = argv[3];
  if (environment == "sim")
    motor_key = "simulated_motors";
  else if (environment == "real")
    motor_key = "real_motors";

  for (auto motor : config[motor_key]) {
    std::string s = motor.as<std::string>();
    lista_subscribers.push_back(s);
  }

  pub_joint_state_ros1 = ros1_node.advertise<sensor_msgs::JointState>("/joint_states", 10);

  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("mara_bridge");

  std::vector<FollowJointTrajectoryAction*> lista_follow_joint_trajectory_action;

  for(unsigned int i = 0; i < lista_subscribers.size(); i++){
    printf("Creating FollowJointTrajectoryAction %s\n", lista_subscribers[i].c_str());
    lista_follow_joint_trajectory_action.push_back(new FollowJointTrajectoryAction(lista_subscribers[i], ros1_node, ros2_node));
  }

  auto sub_servoMotorGoal = ros2_node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/mara_controller/state", rclcpp::SensorDataQoS(), motorStateCallback);

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
