#include <mara_gazebo_plugins/mara_gazebo_joint_plugin.hpp>

namespace gazebo_plugins
{

MARAGazeboPluginRos::MARAGazeboPluginRos()
: impl_(std::make_unique<MARAGazeboPluginRosPrivate>())
{
}

MARAGazeboPluginRos::~MARAGazeboPluginRos()
{
}

void MARAGazeboPluginRosPrivate::timer_motor_state_msgs()
{
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();

  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg;
  motor_state_msg.header.stamp.sec = cur_time.sec;
  motor_state_msg.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg.position = joints_[MARAGazeboPluginRosPrivate::AXIS1]->Position(0);
  motor_state_msg.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetVelocity(0);
  motor_state_msg.effort = joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetForce(0);
  motor_state_msg.goal = goal_position_axis1_rad;
  motor_state_msg.error = (goal_position_axis1_rad - motor_state_msg.position);
  motor_state_msg.load = 0;
  motor_state_msg.moving = executing_axis1;
  motor_state_msg.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis1_pub->publish(motor_state_msg);

  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis2;
  motor_state_msg_axis2.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis2.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis2.position = joints_[MARAGazeboPluginRosPrivate::AXIS2]->Position(0);
  motor_state_msg_axis2.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetVelocity(0);
  motor_state_msg_axis2.effort = joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetForce(0);
  motor_state_msg_axis2.goal =  goal_position_axis2_rad;
  motor_state_msg_axis2.error = (goal_position_axis2_rad - motor_state_msg_axis2.position);
  motor_state_msg_axis2.load = 0;
  motor_state_msg_axis2.moving = executing_axis2;
  motor_state_msg_axis2.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis2_pub->publish(motor_state_msg_axis2);
}

void MARAGazeboPluginRos::createGenericTopics(std::string node_name)
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

  impl_->info_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::ID>(topic_name_info,
                                            rmw_qos_profile_default);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_info.c_str());

  impl_->power_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::Power>(topic_name_power,
                                            rmw_qos_profile_default);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_power.c_str());

  impl_->status_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::Status>(topic_name_status,
                                            rmw_qos_profile_default);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_status.c_str());

  rmw_qos_profile_t custom_qos_profile;
  custom_qos_profile.depth = 1;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  impl_->sim3d_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::Simulation3D>(topic_name_sim3d,
                custom_qos_profile);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_sim3d.c_str());

  impl_->sim_urdf_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::SimulationURDF>(topic_name_simurdf,
                custom_qos_profile);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_simurdf.c_str());

  impl_->specs_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::SpecsRotaryServo>(topic_name_specs,
                rmw_qos_profile_default);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_specs.c_str());

  impl_->state_comm_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::StateCommunication>(topic_name_state_comm,
                rmw_qos_profile_default);
  RCLCPP_ERROR(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_state_comm.c_str());

  impl_->specs_comm_pub = impl_->ros_node_->create_publisher<hrim_generic_msgs::msg::SpecsCommunication>(topic_name_specs_comm,
                rmw_qos_profile_default);
  RCLCPP_ERROR(impl_->ros_node_->get_logger(), "creating %s publisher topic", topic_name_specs_comm.c_str());

  impl_->timer_info_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&MARAGazeboPluginRosPrivate::timer_info_msgs, impl_.get()));
  impl_->timer_status_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&MARAGazeboPluginRosPrivate::timer_status_msgs, impl_.get()));
  impl_->timer_power_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&MARAGazeboPluginRosPrivate::timer_power_msgs, impl_.get()));
  impl_->timer_specs_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&MARAGazeboPluginRosPrivate::timer_specs_msgs, impl_.get()));
  impl_->timer_comm_ = impl_->ros_node_->create_wall_timer(
      1s, std::bind(&MARAGazeboPluginRosPrivate::timer_comm_msgs, impl_.get()));
}

void MARAGazeboPluginRosPrivate::commandCallback_axis1(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis1){
    if(msg->velocity!=0.0){
      trajectories_position_axis1.clear();
      trajectories_velocities_axis1.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS1]->Position(0);

      double start_time = 0;
      double end_time = fabs(current_pose_rad-msg->position)/msg->velocity;

      Y_vel[0] = 0;
      Y_pos[0] = current_pose_rad;
      X[0] = start_time;

      Y_vel[1] = 0;
      Y_pos[1] = msg->position;
      X[1] = end_time;
      tk::spline interpolation_vel, interpolation_pos;
      if(!interpolation_vel.set_points(X, Y_vel))
        return;
      if(!interpolation_pos.set_points(X, Y_pos))
        return;

      for(double t = start_time; t < end_time; t+= 0.001 ){
        trajectories_position_axis1.push_back(interpolation_pos(t));
        trajectories_velocities_axis1.push_back(interpolation_vel(t));
      }
    }
  }
}

void MARAGazeboPluginRosPrivate::commandCallback_axis2(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis2){
    if(msg->velocity!=0.0){
      trajectories_position_axis2.clear();
      trajectories_velocities_axis2.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS2]->Position(0);

      double start_time = 0;
      double end_time = fabs(current_pose_rad-msg->position)/msg->velocity;

      Y_vel[0] = 0;
      Y_pos[0] = current_pose_rad;
      X[0] = start_time;

      Y_vel[1] = 0;
      Y_pos[1] = msg->position;
      X[1] = end_time;

      tk::spline interpolation_vel, interpolation_pos;
      if(!interpolation_vel.set_points(X, Y_vel))
        return;
      if(!interpolation_pos.set_points(X, Y_pos))
        return;

      for(double t = start_time; t < end_time; t+= 0.001 ){
        trajectories_position_axis2.push_back(interpolation_pos(t));
        trajectories_velocities_axis2.push_back(interpolation_vel(t));
      }
    }
  }
}

void MARAGazeboPluginRosPrivate::trajectoryAxis1Callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if(!executing_axis1){
    std::vector<double> X(msg->points.size()), Y_vel(msg->points.size()), Y_pos(msg->points.size());
    for(unsigned int point = 0; point < msg->points.size(); point++){
      double start_time = msg->points[point].time_from_start.sec +
                          msg->points[point].time_from_start.nanosec/NSEC_PER_SECOND;
      Y_vel[point] =  msg->points[point].velocities[0];
      Y_pos[point] =  msg->points[point].positions[0];
      X[point] = start_time;
    }
    double start_time = 0;
    double end_time = msg->points[msg->points.size()-1].time_from_start.sec +
                      msg->points[msg->points.size()-1].time_from_start.nanosec/NSEC_PER_SECOND;

    tk::spline interpolation_vel, interpolation_pos;
    if(!interpolation_vel.set_points(X, Y_vel))
      return;
    if(!interpolation_pos.set_points(X, Y_pos))
      return;

    int index_x = 1;
    for(double t = start_time; t < end_time; t+=0.001 ){

      double time_point = msg->points[index_x].time_from_start.sec +
                          msg->points[index_x].time_from_start.nanosec/NSEC_PER_SECOND;
      if(t > time_point)
        index_x++;
      trajectories_position_axis1.push_back(interpolation_pos(t));
      trajectories_velocities_axis1.push_back(interpolation_vel(t));
    }
  }
}

void MARAGazeboPluginRosPrivate::trajectoryAxis2Callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if(!executing_axis2){
    std::vector<double> X(msg->points.size()), Y_vel(msg->points.size()), Y_pos(msg->points.size());
    for(unsigned int point = 0; point < msg->points.size(); point++){
      double start_time = msg->points[point].time_from_start.sec +
                          msg->points[point].time_from_start.nanosec/NSEC_PER_SECOND;
      Y_vel[point] =  msg->points[point].velocities[0];
      Y_pos[point] =  msg->points[point].positions[0];
      X[point] = start_time;
    }
    double start_time = 0;
    double end_time = msg->points[msg->points.size()-1].time_from_start.sec +
                      msg->points[msg->points.size()-1].time_from_start.nanosec/NSEC_PER_SECOND;

    tk::spline interpolation_vel, interpolation_pos;
    if(!interpolation_vel.set_points(X, Y_vel))
      return;
    if(!interpolation_pos.set_points(X, Y_pos))
      return;

    int index_x = 1;
    for(double t=start_time; t < end_time; t+=0.001 ){

      double time_point = msg->points[index_x].time_from_start.sec +
                          msg->points[index_x].time_from_start.nanosec/NSEC_PER_SECOND;
      if(t > time_point)
        index_x++;
      trajectories_position_axis2.push_back(interpolation_pos(t));
      trajectories_velocities_axis2.push_back(interpolation_vel(t));
    }
  }
}

void MARAGazeboPluginRos::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->trajectories_position_axis1.clear();
  impl_->trajectories_velocities_axis1.clear();
  impl_->trajectories_position_axis2.clear();
  impl_->trajectories_velocities_axis2.clear();
  impl_->executing_axis1 = false;
  impl_->executing_axis2 = false;
  impl_->index_trajectory_axis1 = 0;
  impl_->index_trajectory_axis2 = 0;
  impl_->goal_position_axis1_rad = 0;
  impl_->goal_position_axis2_rad = 0;

  std::string node_name = _sdf->Get<std::string>("name");
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "name %s\n", node_name.c_str());

  createGenericTopics(node_name);

  // Get joints
  impl_->joints_.resize(2);

  auto motor1 = _sdf->Get<std::string>("axis1", "axis1").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS1] = _model->GetJoint(motor1);

  auto motor2 = _sdf->Get<std::string>("axis2", "axis2").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS2] = _model->GetJoint(motor2);

  if (!impl_->joints_[MARAGazeboPluginRosPrivate::AXIS1] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS2])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s] or [%s] not found, plugin will not work.", motor1, motor2);
    impl_->ros_node_.reset();
    return;
  }

  // Creating motor state topic name
  std::string topic_name_motor_state = std::string(node_name) + "/state";
  impl_->motor_state_axis1_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state.c_str() );

  std::string topic_name_motor_state_axis2 = std::string(node_name) + "2/state";
  impl_->motor_state_axis2_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis2,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis2.c_str() );

  // Creating command topic name
  std::string topic_command_state = std::string(node_name) + "/goal";
  impl_->command_sub_axis1_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis1, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state.c_str() );


  std::string topic_command_state_axis2 = std::string(node_name) + "2/goal";
  impl_->command_sub_axis2_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis2,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis2, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state.c_str() );


  std::string topic_trajectory_axis1 = std::string(node_name) + "/trajectory";
  impl_->trajectory_sub_ = impl_->ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
         topic_trajectory_axis1,
         std::bind(&MARAGazeboPluginRosPrivate::trajectoryAxis1Callback, impl_.get(), std::placeholders::_1),
         rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_trajectory_axis1.c_str() );

  std::string topic_trajectory_axis2 = std::string(node_name) + "2/trajectory";

  impl_->trajectory2_sub_ = impl_->ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          topic_trajectory_axis2,
          std::bind(&MARAGazeboPluginRosPrivate::trajectoryAxis2Callback, impl_.get(), std::placeholders::_1),
          rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_trajectory_axis2.c_str() );

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 1000.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Update rate: %.4f\n", impl_->update_period_);

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&MARAGazeboPluginRosPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  impl_->timer_motor_state_ = impl_->ros_node_->create_wall_timer(
        50ms, std::bind(&MARAGazeboPluginRosPrivate::timer_motor_state_msgs, impl_.get()));
}

void MARAGazeboPluginRos::Reset()
{

}

void MARAGazeboPluginRosPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  if(!executing_axis1 && trajectories_position_axis1.size()>0){
    index_trajectory_axis1 = 0;
    executing_axis1 = true;
  }

  if(executing_axis1){
    goal_position_axis1_rad = trajectories_position_axis1[index_trajectory_axis1];
    index_trajectory_axis1++;
    if(index_trajectory_axis1==trajectories_position_axis1.size()){
      executing_axis1 = false;
      trajectories_position_axis1.clear();
      index_trajectory_axis1 = 0;
    }
  }

  if(!executing_axis2 && trajectories_position_axis2.size()>0){
    index_trajectory_axis2 = 0;
    executing_axis2 = true;
  }

  if(executing_axis2){
    goal_position_axis2_rad = trajectories_position_axis2[index_trajectory_axis2];
    index_trajectory_axis2++;
    if(index_trajectory_axis2==trajectories_position_axis2.size()){
      executing_axis2 = false;
      trajectories_position_axis2.clear();
      index_trajectory_axis2 = 0;
    }
  }

  joints_[MARAGazeboPluginRosPrivate::AXIS1]->SetPosition(0, goal_position_axis1_rad, false);
  joints_[MARAGazeboPluginRosPrivate::AXIS2]->SetPosition(0, goal_position_axis2_rad, false);

  last_update_time_ = _info.simTime;
}

void MARAGazeboPluginRosPrivate::timer_info_msgs()
{
  hrim_generic_msgs::msg::ID info_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  info_msg.header.stamp.sec = cur_time.sec;
  info_msg.header.stamp.nanosec = cur_time.nsec;
  info_msg.device_kind_id = hrim_generic_msgs::msg::ID::HRIM_SENSOR;
  info_msg.hros_version = "Ardent";
  info_msg.hrim_version = "Anboto";
  info_pub->publish(info_msg);
}

void MARAGazeboPluginRosPrivate::timer_power_msgs()
{
  hrim_generic_msgs::msg::Power power_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  power_msg.header.stamp.sec = cur_time.sec;
  power_msg.header.stamp.nanosec = cur_time.nsec;
  power_msg.voltage = 48.0;
  power_msg.current_consumption = 0.1;
  power_msg.power_consumption = power_msg.current_consumption*power_msg.voltage;
  power_pub->publish(power_msg);
}

void MARAGazeboPluginRosPrivate::timer_status_msgs()
{
  hrim_generic_msgs::msg::Status status_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  status_msg.header.stamp.sec = cur_time.sec;
  status_msg.header.stamp.nanosec = cur_time.nsec;
  status_pub->publish(status_msg);
}

void MARAGazeboPluginRosPrivate::timer_specs_msgs()
{
  hrim_actuator_rotaryservo_msgs::msg::SpecsRotaryServo specs_msg;
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
  specs_msg.header.stamp.sec = cur_time.sec;
  specs_msg.header.stamp.nanosec = cur_time.nsec;
  specs_msg.control_type = (uint8_t)hrim_actuator_rotaryservo_msgs::msg::SpecsRotaryServo::CONTROL_TYPE_POSITION_VELOCITY;
  specs_msg.range_min = -6.27; // multi-turn absolute +/-4 tuens
  specs_msg.range_max = 6.27;
  specs_msg.precision = 0.00008722222; // 0.005º

  specs_msg.rated_speed = 1.46607657; // 14 RPM
  specs_msg.reachable_speed = 1.46607657; // 14 RPM
  specs_msg.rated_torque = 9; // 9-Nm
  specs_msg.reachable_torque = 13; // 13-Nm

  specs_msg.temperature_range_min  = -10.0; // -10º
  specs_msg.temperature_range_max  = +50.0; // 50º
  specs_pub->publish(specs_msg);
}

void MARAGazeboPluginRosPrivate::timer_comm_msgs()
{
  gazebo::common::Time cur_time = model_->GetWorld()->SimTime();

  hrim_generic_msgs::msg::StateCommunication state_comm_msg;
  state_comm_msg.header.stamp.sec = cur_time.sec;
  state_comm_msg.header.stamp.nanosec = cur_time.nsec;
  state_comm_pub->publish(state_comm_msg);

  hrim_generic_msgs::msg::SpecsCommunication specs_comm_msg;
  specs_comm_msg.header.stamp.sec = cur_time.sec;
  specs_comm_msg.header.stamp.nanosec = cur_time.nsec;
  specs_comm_pub->publish(specs_comm_msg);

}

GZ_REGISTER_MODEL_PLUGIN(MARAGazeboPluginRos)
}  // namespace gazebo_plugins
