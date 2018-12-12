#include "hros_cognition_mara_components/HROSCognitionMaraComponents.hpp"

HROSCognitionMaraComponentsNode::HROSCognitionMaraComponentsNode(const std::string & node_name,
                   int argc, char **argv, bool intra_process_comms )
: rclcpp_lifecycle::LifecycleNode(node_name, "", intra_process_comms)
{

  this->node_name = node_name;

  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
                                this->node_name + std::string("/get_state"));
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
                                this->node_name + std::string("/change_state"));
  auto qos_state = rmw_qos_profile_sensor_data;
  qos_state.depth = 1;
  common_joints_pub_ = create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
    		 "/mara_controller/state",
         qos_state);

  trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
         "/mara_controller/command",
         std::bind(&HROSCognitionMaraComponentsNode::commandCallback, this, _1),
         rmw_qos_profile_sensor_data);

  if (rcutils_cli_option_exist(argv, argv + argc, "-motors")){
    file_motors = std::string(rcutils_cli_get_option(argv, argv + argc, "-motors"));
  }

  pthread_mutex_init(&mtx, NULL);
  pthread_mutex_init(&mutex_command, NULL);

  nan = std::numeric_limits<float>::quiet_NaN();
}

void HROSCognitionMaraComponentsNode::init_state_machine()
{
  unsigned int state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

  while(state!=lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      state = trigger_transition(
        rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)).id();

  while(state!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      state = trigger_transition(
        rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)).id();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is called.");

	std::vector<std::string> node_names;

  std::cout << "===================== Reading link order ========================" << std::endl;
  std::vector<std::string> topic_order;
  std::cout << "Trying to open  " << file_motors << std::endl;

  YAML::Node config = YAML::LoadFile(file_motors);
  if(config.IsNull()){
      std::cout << "Not able to open the file" << std::endl;
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  std::vector<std::string> lista_subcribers;

  for (auto motor : config["motors"]) {
    std::string s = motor.as<std::string>();
    topic_order.push_back(s);
    std::cout << "topic name: " << s << std::endl;
  }
  std::cout << "=====================================================" << std::endl;

  std::cout << "++++++++++++++++++ Subscriptions ++++++++++++++++++" << std::endl;
  for(unsigned int i = 0; i < topic_order.size(); i++){
    std::string name_motor = std::string("/") + topic_order[i];
    name_motor = topic_order[i];
    auto subscriber = this->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
                              std::string("/") + topic_order[i] + "/state",
                              [this, name_motor](hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg) {
                                stateCallback(name_motor, msg->velocity, msg->position, msg->effort);
                              },rmw_qos_profile_sensor_data);
    motor_state_subcriptions_.push_back(subscriber);
    std::cout << "Subscribe at " << topic_order[i] << " " << subscriber->get_topic_name() << std::endl;
  }

  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << std::endl << std::endl;

  std::cout << "----------------- Publishers -----------------" << std::endl;

  for(unsigned int i = 0; i < topic_order.size(); i++){
    auto publisher_command = this->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(
                                   std::string("/") + topic_order[i] + "/goal",
                                   rmw_qos_profile_sensor_data);
    motor_goal_publishers_.push_back(publisher_command);
    std::cout << "New publisher at " <<  std::string("/") + topic_order[i] + "/goal" << std::endl;
  }

  msg_actuators_.actual.positions.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.velocities.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.effort.resize(motor_goal_publishers_.size());
  msg_actuators_.joint_names.resize(motor_goal_publishers_.size());

  for(unsigned int i = 0; i < topic_order.size(); i++){
    msg_actuators_.joint_names[i] =  topic_order[i];
  }

  timer_common_joints_ = this->create_wall_timer(
      20ms, std::bind(&HROSCognitionMaraComponentsNode::timer_stateCommonPublisher, this));
  timer_command_ = this->create_wall_timer(
      10ms, std::bind(&HROSCognitionMaraComponentsNode::timer_commandPublisher, this));

  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is finished.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_activate() is called.");

  common_joints_pub_->on_activate();

  for(unsigned int i = 0; i < motor_goal_publishers_.size(); i++){
    motor_goal_publishers_[i]->on_activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_deactivate() is called.");

  common_joints_pub_->on_deactivate();

  for(unsigned int i = 0; i < motor_goal_publishers_.size(); i++){
    motor_goal_publishers_[i]->on_deactivate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_cleanup() is called.");

  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  timer_common_joints_.reset();

  motor_goal_publishers_.clear();

  cmd_to_send.clear();

  msg_actuators_.joint_names.clear();
  msg_actuators_.actual.positions.clear();
  msg_actuators_.actual.velocities.clear();
  msg_actuators_.actual.effort.clear();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_shutdown() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HROSCognitionMaraComponentsNode::on_error(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_error() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
