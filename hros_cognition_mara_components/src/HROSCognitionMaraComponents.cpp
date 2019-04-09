#include "hros_cognition_mara_components/HROSCognitionMaraComponents.hpp"

HROSCognitionMaraComponentsNode::HROSCognitionMaraComponentsNode(const std::string & node_name,
                   int argc, char **argv, bool intra_process_comms )
: rclcpp::Node(node_name, "", intra_process_comms)
{

  this->node_name = node_name;

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

  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is called.");

	std::vector<std::string> node_names;

  std::cout << "===================== Reading link order ========================" << std::endl;
  std::vector<std::string> topic_order;
  std::cout << "Trying to open  " << file_motors << std::endl;

  YAML::Node config = YAML::LoadFile(file_motors);
  if(config.IsNull()){
      std::cout << "Not able to open the file" << std::endl;
      return;
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
                              std::string("/") + topic_order[i],
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
                                   std::string("/") + topic_order[i],
                                   rmw_qos_profile_sensor_data);
    motor_goal_publishers_.push_back(publisher_command);
    std::cout << "New publisher at " <<  std::string("/") + topic_order[i] << std::endl;
  }

  msg_actuators_.actual.positions.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.velocities.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.effort.resize(motor_goal_publishers_.size());
  msg_actuators_.joint_names.resize(motor_goal_publishers_.size());

  for(unsigned int i = 0; i < topic_order.size(); i++){
    msg_actuators_.joint_names[i] =  topic_order[i];
  }

  msg_actuators_callback_sync.resize(topic_order.size());
  for(unsigned int i = 0; i < msg_actuators_callback_sync.size(); i++){
    msg_actuators_callback_sync[i] =  false;
  }

  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is finished.");
}
