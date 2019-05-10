#include "HRIMCognitionMaraComponents.hpp"

HRIMCognitionMaraComponentsNode::HRIMCognitionMaraComponentsNode(const std::string & node_name,
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
         std::bind(&HRIMCognitionMaraComponentsNode::commandCallback, this, _1),
         rmw_qos_profile_sensor_data);

  if (rcutils_cli_option_exist(argv, argv + argc, "-mara")){
    file_motors = std::string(rcutils_cli_get_option(argv, argv + argc, "-mara"));
  }

  pthread_mutex_init(&mtx, NULL);
  pthread_mutex_init(&mutex_command, NULL);

  nan = std::numeric_limits<float>::quiet_NaN();

  RCUTILS_LOG_INFO_NAMED(get_name(), "HRIMCognitionMaraComponentsNode::on_configure() is called.");

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

  environment = argv[3];
  if (environment == "sim")
    motor_key = "simulated_motors";
  else if (environment == "real")
    motor_key = "real_motors";

  for (auto motor : config[motor_key]) {
    std::string s = motor.as<std::string>();
    topic_order.push_back(s);
    std::cout << "topic name: " << s << std::endl;
  }
  std::cout << "=====================================================" << std::endl;

  std::cout << "++++++++++++++++++ Subscribers and Publishers++++++++++++++++++" << std::endl;
  for(unsigned int i = 0; i < topic_order.size(); i++){

    std::string topic = topic_order[i];
    std::string delimiter = "trajectory";
    std::string id = topic.substr( 0, topic.find(delimiter) );
    std::string axis = topic.erase( 0, topic.find(delimiter) + delimiter.length() );
    std::string motor_name = id + "state" + axis;

    auto subscriber = this->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
                              motor_name,
                              [this, motor_name](hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg) {
                                stateCallback(motor_name, msg->velocity, msg->position, msg->effort);
                              },rmw_qos_profile_sensor_data);
    motor_state_subcriptions_.push_back(subscriber);
    std::cout << "Subscribe at " << motor_name << std::endl;

    motor_name = id + "goal" + axis;

    auto publisher_command = this->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(motor_name, rmw_qos_profile_sensor_data);
    motor_goal_publishers_.push_back(publisher_command);
    std::cout << "New publisher at " << motor_name << std::endl;
  }

  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

  msg_actuators_.actual.positions.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.velocities.resize(motor_goal_publishers_.size());
  msg_actuators_.actual.effort.resize(motor_goal_publishers_.size());
  msg_actuators_.joint_names.resize(motor_goal_publishers_.size());

  for(unsigned int i = 0; i < topic_order.size(); i++){

    std::string topic = topic_order[i];
    std::string delimiter = "trajectory";
    std::string id = topic.substr( 0, topic.find(delimiter) );
    std::string axis = topic.erase( 0, topic.find(delimiter) + delimiter.length() );
    std::string motor_name = id + "state" + axis;

    msg_actuators_.joint_names[i] =  motor_name;

    std::cout << "motor_name: " << motor_name << std::endl;
  }

  std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << std::endl << std::endl;

  msg_actuators_callback_sync.resize(topic_order.size());
  for(unsigned int i = 0; i < msg_actuators_callback_sync.size(); i++){
    msg_actuators_callback_sync[i] =  false;
  }

  RCUTILS_LOG_INFO_NAMED(get_name(), "HRIMCognitionMaraComponentsNode::on_configure() is finished.");
}
