#include <string>
#include <vector>

#include "robotiq_arg2f_model_articulated_gazebo_plugins/Robotiq140Plugin.h"

namespace gazebo
  {
  ////////////////////////////////////////////////////////////////////////////////
  RobotiqHandPlugin::RobotiqHandPlugin()
  {
    printf("RobotiqHandPlugin\n");
  }

  ////////////////////////////////////////////////////////////////////////////////
  RobotiqHandPlugin::~RobotiqHandPlugin()
  {
    // gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  }

  ////////////////////////////////////////////////////////////////////////////////
  bool RobotiqHandPlugin::IsHandFullyOpen()
  {
    bool fingersOpen = true;

    // The hand will be fully open when all the fingers are within 'tolerance'
    // from their lower limits.
    ignition::math::Angle tolerance;
    tolerance.Degree(1.0);

    fingersOpen = fingersOpen && (left_inner_knuckle_joint->Position(0) < (left_inner_knuckle_joint->LowerLimit(0) + tolerance.Radian()));
    fingersOpen = fingersOpen && (right_inner_knuckle_joint->Position(0) < (right_inner_knuckle_joint->LowerLimit(0) + tolerance.Radian()));

    return fingersOpen;
  }

  ////////////////////////////////////////////////////////////////////////////////
  uint8_t RobotiqHandPlugin::GetCurrentPosition(
    const gazebo::physics::JointPtr &_joint)
  {
    // Full range of motion.
    ignition::math::Angle range =
      _joint->UpperLimit(0) - _joint->LowerLimit(0);

    // Angle relative to the lower limit.
    ignition::math::Angle relAngle = _joint->Position(0) - _joint->LowerLimit(0);

    return
      static_cast<uint8_t>(round(255.0 * relAngle.Radian() / range.Radian()));
  }

  void RobotiqHandPlugin::gripper_service(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request,
        std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response)
  {
    (void)request_header;

    targetPose_right = request->goal_angularposition;//right_inner_knuckle_joint->UpperLimit(0);
    targetPose_left = request->goal_angularposition;//left_inner_knuckle_joint->UpperLimit(0);

    double currentPose_right = right_inner_knuckle_joint->Position(0);
    double currentPose_left = left_inner_knuckle_joint->Position(0);

    if(currentPose_right - targetPose_right > 0)
      sentido = 1;
    else
      sentido = -1;

    posePID_left_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_left_inner_knuckle.SetCmd(0.0);
    posePID_right_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_right_inner_knuckle.SetCmd(0.0);

    sentido = sentido*-1;

    if(sentido>0){
      posePID_left_inner_knuckle.SetCmdMin(0);
      posePID_left_inner_knuckle.SetCmdMax(cmdmax);
      posePID_right_inner_knuckle.SetCmdMin(0);
      posePID_right_inner_knuckle.SetCmdMax(cmdmax);
    }else{
      posePID_left_inner_knuckle.SetCmdMin(cmdmin);
      posePID_left_inner_knuckle.SetCmdMax(0);
      posePID_right_inner_knuckle.SetCmdMin(cmdmin);
      posePID_right_inner_knuckle.SetCmdMax(0);
    }

    gzmsg << "Position PID parameters for joints "  << std::endl
          << "\tKP: "     << posePID_left_inner_knuckle.GetPGain()  << std::endl
          << "\tKI: "     << posePID_left_inner_knuckle.GetIGain()  << std::endl
          << "\tKD: "     << posePID_left_inner_knuckle.GetDGain()  << std::endl
          << "\tIMin: "   << posePID_left_inner_knuckle.GetIMin()   << std::endl
          << "\tIMax: "   << posePID_left_inner_knuckle.GetIMax()   << std::endl
          << "\tCmdMin: " << posePID_left_inner_knuckle.GetCmdMin() << std::endl
          << "\tCmdMax: " << posePID_left_inner_knuckle.GetCmdMax() << std::endl
          << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void RobotiqHandPlugin::Load(gazebo::physics::ModelPtr _parent,
                               sdf::ElementPtr _sdf)
  {
    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    std::string node_name = _sdf->Get<std::string>("name");
    RCLCPP_INFO(ros_node_->get_logger(), "name %s\n", node_name.c_str());

    createGenericTopics(node_name);

    std::string robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
       robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    else{
      printf("No robotNamespace\n");
    }

    ros_node_->set_parameters({
      rclcpp::Parameter("kp_gripper", kp),
      rclcpp::Parameter("ki_gripper", ki),
      rclcpp::Parameter("kd_gripper", kd),
      rclcpp::Parameter("imin_gripper", imin),
      rclcpp::Parameter("imax_gripper", imax),
      rclcpp::Parameter("cmdmin_gripper", cmdmin),
      rclcpp::Parameter("cmdmax_gripper", cmdmax)
    });

    auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        rclcpp::ParameterType parameter_type = parameter.get_type();

        int error = 0;

        if(!parameter.get_name().compare("kp_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->kp = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("ki_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->ki = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("kd_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->kd = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("imin_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->imin = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("imax_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->imax = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("cmdmin_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->cmdmin = parameter.as_double();
          }else{
            error = 1;
          }
        }else if(!parameter.get_name().compare("cmdmax_gripper")){
          if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
            this->cmdmax = parameter.as_double();
          }else{
            error = 1;
          }
        }else{
          error = 2;
        }

        if(error==1){
          RCLCPP_INFO(ros_node_->get_logger(),
            "requested value for parameter '%s' is not the right type",
            parameter.get_name().c_str());
            result.successful = false;
        }
        if(error==2){
          RCLCPP_INFO(ros_node_->get_logger(), "Parameter %s doesn't exit",   parameter.get_name().c_str());
          result.successful = false;
        }
      }
    };
    ros_node_->register_param_change_callback(param_change_callback);

    printf("RobotiqHandPlugin::Load\n");
    gzmsg << "RobotiqHandPlugin::Load" << std::endl;
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->sdf = _sdf;

    // Error message if the model couldn't be found
    if (!this->model){
      gzerr<< "Parent model is NULL! RobotiqHandPlugin could not be loaded."<< std::endl;
      return;
    }
    gzmsg<< "robot_namespace_ " << robot_namespace_ << std::endl;

    gzmsg<< "_sdf description " << _sdf->GetDescription() << std::endl;

    for(unsigned int i = 0; i < model->GetJoints().size(); i++){
        gzmsg << this->model->GetJoints()[i]->GetScopedName() << std::endl;
    }

    posePID_left_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_right_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);

    left_inner_knuckle_joint = this->model->GetJoint("left_inner_knuckle_joint");
    if (!left_inner_knuckle_joint){
      gzthrow("could not find front left_inner_knuckle_joint\n");
    }
    this->model->GetJointController()->SetPositionPID(
        this->left_inner_knuckle_joint->GetScopedName(), this->posePID_left_inner_knuckle);

    gzmsg << "left_inner_knuckle_joint LowerLimit " << left_inner_knuckle_joint->LowerLimit(0) << std::endl;
    gzmsg << "left_inner_knuckle_joint UpperLimit " << left_inner_knuckle_joint->UpperLimit(0) << std::endl;
    gzmsg << "left_inner_knuckle_joint GetEffortLimit " << left_inner_knuckle_joint->GetEffortLimit(0) << std::endl;
    gzmsg << "left_inner_knuckle_joint GetVelocityLimit " << left_inner_knuckle_joint->GetVelocityLimit(0) << std::endl;
    posePID_left_inner_knuckle.SetCmdMin(-left_inner_knuckle_joint->GetEffortLimit(0));
    posePID_left_inner_knuckle.SetCmdMax(left_inner_knuckle_joint->GetEffortLimit(0));

    right_inner_knuckle_joint = this->model->GetJoint("right_inner_knuckle_joint");
    if (!right_inner_knuckle_joint){
      gzthrow("could not find front right_inner_knuckle_joint\n");
    }
    this->model->GetJointController()->SetPositionPID(
        this->right_inner_knuckle_joint->GetScopedName(), this->posePID_right_inner_knuckle);

    gzmsg << "right_inner_knuckle_joint LowerLimit " << right_inner_knuckle_joint->LowerLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint UpperLimit " << right_inner_knuckle_joint->UpperLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint GetEffortLimit " << right_inner_knuckle_joint->GetEffortLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint GetVelocityLimit " << right_inner_knuckle_joint->GetVelocityLimit(0) << std::endl;
    posePID_right_inner_knuckle.SetCmdMin(-right_inner_knuckle_joint->GetEffortLimit(0));
    posePID_right_inner_knuckle.SetCmdMax(right_inner_knuckle_joint->GetEffortLimit(0));

    this->lastControllerUpdateTime = this->world->SimTime();

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request>,
                        std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response>)> cb_fingercontrol_function = std::bind(
          &RobotiqHandPlugin::gripper_service, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

    srv_ = ros_node_->create_service<hrim_actuator_gripper_srvs::srv::ControlFinger>(node_name + "/goal", cb_fingercontrol_function);

    std::string topic_name_specs = std::string(node_name) + "/specs";
    specs_pub = ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::SpecsFingerGripper>(topic_name_specs,
                  rmw_qos_profile_default);

    std::string topic_name_gripper_state = std::string(node_name) + "/state";
    gripper_state_pub = ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>(topic_name_gripper_state,
                  rmw_qos_profile_default);

    // Connect to gazebo world update.
    this->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotiqHandPlugin::UpdateStates, this));

    sentido = -1;

  }

  void RobotiqHandPlugin::UpdateStates()
  {
    gazebo::common::Time curTime = this->world->SimTime();

    // Update the hand controller.
    this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());

    this->lastControllerUpdateTime = curTime;
  }

  void RobotiqHandPlugin::UpdatePIDControl(double _dt)
  {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetPositionTarget(
            this->right_inner_knuckle_joint->GetScopedName(), targetPose_right);
    this->model->GetJointController()->SetPositionTarget(
            this->left_inner_knuckle_joint->GetScopedName(), targetPose_left);
  }

  void RobotiqHandPlugin::createGenericTopics(std::string node_name)
  {
    // create info topic
    std::string topic_name_info = std::string(node_name) + "/id";

    // Creating status topic name
    std::string topic_name_status = std::string(node_name) + "/status";

    // Creating power topic name
    std::string topic_name_power = std::string(node_name) + "/power";

    // Creating sim topic name
    std::string topic_name_sim3d = std::string(node_name) + "/module_3d";
    std::string topic_name_simurdf = std::string(node_name) + "/module_urdf";
    std::string topic_name_specs = std::string(node_name) + "/specs";
    std::string topic_name_specs_comm = std::string(node_name) + "/specs_comm";
    std::string topic_name_state_comm = std::string(node_name) + "/state_comm";

    info_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::ID>(topic_name_info);

    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_info.c_str());

    power_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::Power>(topic_name_power);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_power.c_str());

    status_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::Status>(topic_name_status);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_status.c_str());

    rmw_qos_profile_t custom_qos_profile;
    custom_qos_profile.depth = 1;
    custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    sim3d_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::Simulation3D>(topic_name_sim3d,
                  custom_qos_profile);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_sim3d.c_str());

    sim_urdf_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::SimulationURDF>(topic_name_simurdf,
                  custom_qos_profile);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_simurdf.c_str());

    specs_pub = ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::SpecsFingerGripper>(topic_name_specs,
                  rmw_qos_profile_default);
    RCLCPP_INFO(ros_node_->get_logger(), "creating %s publisher topic", topic_name_specs.c_str());

    state_comm_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::StateCommunication>(topic_name_state_comm,
                  rmw_qos_profile_default);
    RCLCPP_ERROR(ros_node_->get_logger(), "creating %s publisher topic", topic_name_state_comm.c_str());

    specs_comm_pub = ros_node_->create_publisher<hrim_generic_msgs::msg::SpecsCommunication>(topic_name_specs_comm,
                  rmw_qos_profile_default);
    RCLCPP_ERROR(ros_node_->get_logger(), "creating %s publisher topic", topic_name_specs_comm.c_str());

    timer_info_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqHandPlugin::timer_info_msgs, this));
    timer_status_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqHandPlugin::timer_status_msgs, this));
    timer_power_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqHandPlugin::timer_power_msgs, this));
    timer_specs_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqHandPlugin::timer_specs_msgs, this));
    timer_comm_ = ros_node_->create_wall_timer(
        1s, std::bind(&RobotiqHandPlugin::timer_comm_msgs, this));
    timer_gripper_status_ = ros_node_->create_wall_timer(
        100ms, std::bind(&RobotiqHandPlugin::timer_gripper_status_msgs, this));
  }

  void RobotiqHandPlugin::timer_info_msgs()
  {
    hrim_generic_msgs::msg::ID info_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    info_msg.header.stamp.sec = cur_time.sec;
    info_msg.header.stamp.nanosec = cur_time.nsec;
    info_msg.device_kind_id = hrim_generic_msgs::msg::ID::HRIM_SENSOR;
    info_msg.hros_version = "Ardent";
    info_msg.hrim_version = "Anboto";
    info_pub->publish(info_msg);
  }

  void RobotiqHandPlugin::timer_power_msgs()
  {
    hrim_generic_msgs::msg::Power power_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    power_msg.header.stamp.sec = cur_time.sec;
    power_msg.header.stamp.nanosec = cur_time.nsec;
    power_msg.voltage = 48.0;
    power_msg.current_consumption = 0.1;
    power_msg.power_consumption = power_msg.current_consumption*power_msg.voltage;
    power_pub->publish(power_msg);
  }

  void RobotiqHandPlugin::timer_status_msgs()
  {
    hrim_generic_msgs::msg::Status status_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    status_msg.header.stamp.sec = cur_time.sec;
    status_msg.header.stamp.nanosec = cur_time.nsec;
    status_pub->publish(status_msg);
  }

  void RobotiqHandPlugin::timer_gripper_status_msgs()
  {
    hrim_actuator_gripper_msgs::msg::StateFingerGripper state_gripper_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    state_gripper_msg.header.stamp.sec = cur_time.sec;
    state_gripper_msg.header.stamp.nanosec = cur_time.nsec;
    state_gripper_msg.angular_position = right_inner_knuckle_joint->Position(0);
    state_gripper_msg.linear_position = 0;

    gripper_state_pub->publish(state_gripper_msg);

  }

  void RobotiqHandPlugin::timer_specs_msgs()
  {
    hrim_actuator_gripper_msgs::msg::SpecsFingerGripper specs_msg;
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();
    specs_msg.header.stamp.sec = cur_time.sec;
    specs_msg.header.stamp.nanosec = cur_time.nsec;
    specs_msg.min_force = MIN_FORCE; // Minimum gripping force [N]
    specs_msg.max_force = MAX_FORCE; // Maximun gripping force [N]

    specs_msg.max_payload = MAX_FORCE;   // Maximum recommended payload [kg]

    specs_msg.min_speed = MIN_SPEED;  // Minimum closing speed [mm/s]
    specs_msg.max_speed = MAX_SPEED;  // Maximum  closing speed [mm/s]

    specs_msg.max_acceleration = MAX_ACCELERATION;

    specs_msg.max_length = MAX_LENGHT; // Maximum permitted finger length [mm]
    specs_msg.max_angle = MAX_ANGLE;  // Maximum permitted finger angle [rad]

    specs_msg.repeatability = REPEATABILITY;

    specs_pub->publish(specs_msg);
  }

  void RobotiqHandPlugin::timer_comm_msgs()
  {
    gazebo::common::Time cur_time = this->model->GetWorld()->SimTime();

    hrim_generic_msgs::msg::StateCommunication state_comm_msg;
    state_comm_msg.header.stamp.sec = cur_time.sec;
    state_comm_msg.header.stamp.nanosec = cur_time.nsec;
    state_comm_pub->publish(state_comm_msg);

    hrim_generic_msgs::msg::SpecsCommunication specs_comm_msg;
    specs_comm_msg.header.stamp.sec = cur_time.sec;
    specs_comm_msg.header.stamp.nanosec = cur_time.nsec;
    specs_comm_pub->publish(specs_comm_msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqHandPlugin)
}
