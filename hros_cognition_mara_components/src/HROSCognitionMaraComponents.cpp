#include "HROSCognitionMaraComponents.hpp"

HROSCognitionMaraComponentsNode::HROSCognitionMaraComponentsNode(const std::string & node_name, int argc, char **argv): rclcpp::Node(node_name)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is called.");

  this->node_name = node_name;

  common_joints_pub_ = create_publisher<control_msgs::msg::JointTrajectoryControllerState>("/mara_controller/state", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)));

  trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/mara_controller/command", rclcpp::SensorDataQoS(), std::bind(&HROSCognitionMaraComponentsNode::commandCallback, this, _1));

  if (rcutils_cli_option_exist(argv, argv + argc, "-motors")){
    file_motors = std::string(rcutils_cli_get_option(argv, argv + argc, "-motors"));
  }

  pthread_mutex_init(&mtx, NULL);
  pthread_mutex_init(&mutex_command, NULL);

  nan = std::numeric_limits<float>::quiet_NaN();
	std::vector<std::string> node_names;
  std::vector<std::string> topic_order;

  YAML::Node config = YAML::LoadFile(file_motors);
  if(config.IsNull()){
    std::cout << "Not able to open: " << file_motors << std::endl;
    return;
  }
  std::vector<std::string> lista_subcribers;

  environment = argv[3];
  if (environment == "sim")
    motor_key = "simulated_motors";
  else if (environment == "real")
    motor_key = "real_motors";

  std::cout << "====================== Reading link order ======================" << std::endl;
  for (auto motor : config[motor_key]) {
    std::string s = motor.as<std::string>();
    topic_order.push_back(s);
    std::cout << "topic name: " << s << std::endl;
  }
  std::cout << "====================== Name of the motors ======================" << std::endl;
  for(auto ms : config["motors"]){
    std::string m = ms.as<std::string>();
    motor_names.push_back(m);
    std::cout << "motor_names: " << m << std::endl;
  }
  std::cout << "====================== Subscribers and Publishers ======================" << std::endl;
  for(unsigned int i = 0; i < topic_order.size(); i++){

    std::string topic = topic_order[i];
    std::string delimiter = "trajectory";
    std::string id = topic.substr( 0, topic.find(delimiter) );
    std::string axis = topic.erase( 0, topic.find(delimiter) + delimiter.length() );
    std::string motor_name = id + "state" + axis;

    auto subscriber = this->create_subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(
                              motor_name, rclcpp::SensorDataQoS(),
                              [this, motor_name](hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::UniquePtr msg) {
                                stateCallback(motor_name, msg->velocity, msg->position, msg->effort);
                              });
    motor_state_subcriptions_.push_back(subscriber);
    std::cout << "Subscribe at " << motor_name << std::endl;

    motor_name = id + "goal" + axis;

    auto publisher_command = this->create_publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(motor_name, rclcpp::SensorDataQoS(rclcpp::KeepLast(1)));
    motor_goal_publishers_.push_back(publisher_command);
    std::cout << "New publisher at " << motor_name << std::endl;
  }

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

    msg_actuators_.joint_names[i] = motor_name;
  }

  msg_actuators_callback_sync.resize(topic_order.size());
  for(unsigned int i = 0; i < msg_actuators_callback_sync.size(); i++){
    msg_actuators_callback_sync[i] =  false;
  }
  RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::on_configure() is finished.");
}
