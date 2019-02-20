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

  // AXIS 21
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis1;
  motor_state_msg_axis1.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis1.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis1.position = joints_[MARAGazeboPluginRosPrivate::AXIS1]->Position(0);
  motor_state_msg_axis1.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetVelocity(0);
  motor_state_msg_axis1.effort = joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetForce(0);
  motor_state_msg_axis1.goal = goal_position_axis1_rad;
  motor_state_msg_axis1.error = (goal_position_axis1_rad - motor_state_msg_axis1.position);
  motor_state_msg_axis1.load = 0;
  motor_state_msg_axis1.moving = executing_axis1;
  motor_state_msg_axis1.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis1_pub->publish(motor_state_msg_axis1);

  // AXIS 2
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis2;
  motor_state_msg_axis2.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis2.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis2.position = joints_[MARAGazeboPluginRosPrivate::AXIS2]->Position(1);
  motor_state_msg_axis2.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetVelocity(1);
  motor_state_msg_axis2.effort = joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetForce(1);
  motor_state_msg_axis2.goal =  goal_position_axis2_rad;
  motor_state_msg_axis2.error = (goal_position_axis2_rad - motor_state_msg_axis2.position);
  motor_state_msg_axis2.load = 0;
  motor_state_msg_axis2.moving = executing_axis2;
  motor_state_msg_axis2.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis2_pub->publish(motor_state_msg_axis2);

  // AXIS 3
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis3;
  motor_state_msg_axis3.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis3.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis3.position = joints_[MARAGazeboPluginRosPrivate::AXIS3]->Position(2);
  motor_state_msg_axis3.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS3]->GetVelocity(2);
  motor_state_msg_axis3.effort = joints_[MARAGazeboPluginRosPrivate::AXIS3]->GetForce(2);
  motor_state_msg_axis3.goal = goal_position_axis3_rad;
  motor_state_msg_axis3.error = (goal_position_axis3_rad - motor_state_msg_axis3.position);
  motor_state_msg_axis3.load = 0;
  motor_state_msg_axis3.moving = executing_axis3;
  motor_state_msg_axis3.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis3_pub->publish(motor_state_msg_axis3);

  // AXIS 4
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis4;
  motor_state_msg_axis4.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis4.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis4.position = joints_[MARAGazeboPluginRosPrivate::AXIS4]->Position(3);
  motor_state_msg_axis4.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS4]->GetVelocity(3);
  motor_state_msg_axis4.effort = joints_[MARAGazeboPluginRosPrivate::AXIS4]->GetForce(3);
  motor_state_msg_axis4.goal = goal_position_axis4_rad;
  motor_state_msg_axis4.error = (goal_position_axis4_rad - motor_state_msg_axis4.position);
  motor_state_msg_axis4.load = 0;
  motor_state_msg_axis4.moving = executing_axis4;
  motor_state_msg_axis4.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis4_pub->publish(motor_state_msg_axis4);

  // AXIS 5
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis5;
  motor_state_msg_axis5.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis5.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis5.position = joints_[MARAGazeboPluginRosPrivate::AXIS5]->Position(4);
  motor_state_msg_axis5.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS5]->GetVelocity(4);
  motor_state_msg_axis5.effort = joints_[MARAGazeboPluginRosPrivate::AXIS5]->GetForce(4);
  motor_state_msg_axis5.goal = goal_position_axis5_rad;
  motor_state_msg_axis5.error = (goal_position_axis5_rad - motor_state_msg_axis5.position);
  motor_state_msg_axis5.load = 0;
  motor_state_msg_axis5.moving = executing_axis5;
  motor_state_msg_axis5.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis5_pub->publish(motor_state_msg_axis5);

  // AXIS 6
  hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo motor_state_msg_axis6;
  motor_state_msg_axis6.header.stamp.sec = cur_time.sec;
  motor_state_msg_axis6.header.stamp.nanosec = cur_time.nsec;
  motor_state_msg_axis6.position = joints_[MARAGazeboPluginRosPrivate::AXIS6]->Position(5);
  motor_state_msg_axis6.velocity = joints_[MARAGazeboPluginRosPrivate::AXIS6]->GetVelocity(5);
  motor_state_msg_axis6.effort = joints_[MARAGazeboPluginRosPrivate::AXIS6]->GetForce(5);
  motor_state_msg_axis6.goal = goal_position_axis6_rad;
  motor_state_msg_axis6.error = (goal_position_axis6_rad - motor_state_msg_axis6.position);
  motor_state_msg_axis6.load = 0;
  motor_state_msg_axis6.moving = executing_axis6;
  motor_state_msg_axis6.fault = hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo::FAULT_NONE;
  motor_state_axis6_pub->publish(motor_state_msg_axis6);
}

void MARAGazeboPluginRosPrivate::commandCallback_axis1(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis1){
    if(msg->velocity!=0.0){
      UpdateJointPIDs();

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
      UpdateJointPIDs();

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

void MARAGazeboPluginRosPrivate::commandCallback_axis3(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis3){
    if(msg->velocity!=0.0){
      UpdateJointPIDs();

      trajectories_position_axis3.clear();
      trajectories_velocities_axis3.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS3]->Position(0);

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
        trajectories_position_axis3.push_back(interpolation_pos(t));
        trajectories_velocities_axis3.push_back(interpolation_vel(t));
      }
    }
  }
}

void MARAGazeboPluginRosPrivate::commandCallback_axis4(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis4){
    if(msg->velocity!=0.0){
      UpdateJointPIDs();

      trajectories_position_axis4.clear();
      trajectories_velocities_axis4.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS4]->Position(0);

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
        trajectories_position_axis4.push_back(interpolation_pos(t));
        trajectories_velocities_axis4.push_back(interpolation_vel(t));
      }
    }
  }
}

void MARAGazeboPluginRosPrivate::commandCallback_axis5(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis5){
    if(msg->velocity!=0.0){
      UpdateJointPIDs();

      trajectories_position_axis5.clear();
      trajectories_velocities_axis5.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS5]->Position(0);

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
        trajectories_position_axis5.push_back(interpolation_pos(t));
        trajectories_velocities_axis5.push_back(interpolation_vel(t));
      }
    }
  }
}

void MARAGazeboPluginRosPrivate::commandCallback_axis6(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg)
{
  if(!executing_axis6){
    if(msg->velocity!=0.0){
      UpdateJointPIDs();

      trajectories_position_axis6.clear();
      trajectories_velocities_axis6.clear();
      std::vector<double> X(2), Y_vel(2), Y_pos(2);

      float current_pose_rad = joints_[MARAGazeboPluginRosPrivate::AXIS6]->Position(0);

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
        trajectories_position_axis6.push_back(interpolation_pos(t));
        trajectories_velocities_axis6.push_back(interpolation_vel(t));
      }
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

  // Get joints
  impl_->joints_.resize(2);

  auto motor1 = _sdf->Get<std::string>("axis1", "axis1").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS1] = _model->GetJoint(motor1);

  auto motor2 = _sdf->Get<std::string>("axis2", "axis2").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS2] = _model->GetJoint(motor2);

  auto motor3 = _sdf->Get<std::string>("axis3", "axis3").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS3] = _model->GetJoint(motor3);

  auto motor4 = _sdf->Get<std::string>("axis4", "axis4").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS4] = _model->GetJoint(motor4);

  auto motor5 = _sdf->Get<std::string>("axis5", "axis5").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS5] = _model->GetJoint(motor5);

  auto motor6 = _sdf->Get<std::string>("axis6", "axis6").first;
  impl_->joints_[MARAGazeboPluginRosPrivate::AXIS6] = _model->GetJoint(motor6);


  if (!impl_->joints_[MARAGazeboPluginRosPrivate::AXIS1] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS2] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS3] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS4] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS5] ||
    !impl_->joints_[MARAGazeboPluginRosPrivate::AXIS6])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s], [%s], [%s], [%s], [%s] or [%s] not found, plugin will not work.", motor1.c_str(), motor2.c_str(),
      motor3.c_str(), motor4.c_str(), motor5.c_str(), motor6.c_str());
    impl_->ros_node_.reset();
    return;
  }

  // Creating motor state topic name
  // AXIS 1
  std::string topic_name_motor_state_axis1 = std::string(node_name) + "1/state";
  impl_->motor_state_axis1_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis1,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis1.c_str() );

  // AXIS 2
  std::string topic_name_motor_state_axis2 = std::string(node_name) + "12/state";
  impl_->motor_state_axis2_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis2,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis2.c_str() );

  // AXIS 3
  std::string topic_name_motor_state_axis3 = std::string(node_name) + "2/state";
  impl_->motor_state_axis3_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis3,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis3.c_str() );

  // AXIS 4
  std::string topic_name_motor_state_axis4 = std::string(node_name) + "12/state";
  impl_->motor_state_axis4_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis4,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis2.c_str() );

  // AXIS 5
  std::string topic_name_motor_state_axis5 = std::string(node_name) + "1/state";
  impl_->motor_state_axis5_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis5,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis1.c_str() );

  // AXIS 6
  std::string topic_name_motor_state_axis6 = std::string(node_name) + "12/state";
  impl_->motor_state_axis6_pub = impl_->ros_node_->create_publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>(topic_name_motor_state_axis6,
                        rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_name_motor_state_axis2.c_str() );

  // Creating command topic name
  // AXIS 1
  std::string topic_command_state_axis1 = std::string(node_name) + "1/goal";
  impl_->command_sub_axis1_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis1,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis1, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis1.c_str() );

  // AXIS 2
  std::string topic_command_state_axis2 = std::string(node_name) + "12/goal";
  impl_->command_sub_axis2_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis2,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis2, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis2.c_str() );

  // AXIS 3
  std::string topic_command_state_axis3 = std::string(node_name) + "1/goal";
  impl_->command_sub_axis3_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis3,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis1, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis3.c_str() );

  // AXIS 4
  std::string topic_command_state_axis4 = std::string(node_name) + "12/goal";
  impl_->command_sub_axis4_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis4,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis2, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis4.c_str() );

  // AXIS 5
  std::string topic_command_state_axis5 = std::string(node_name) + "1/goal";
  impl_->command_sub_axis5_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis5,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis1, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis5.c_str() );

  // AXIS 6
  std::string topic_command_state_axis6 = std::string(node_name) + "12/goal";
  impl_->command_sub_axis6_ = impl_->ros_node_->create_subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>(topic_command_state_axis6,
                                std::bind(&MARAGazeboPluginRosPrivate::commandCallback_axis2, impl_.get(), std::placeholders::_1),
                                rmw_qos_profile_sensor_data);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating topic %s", topic_command_state_axis6.c_str() );

  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&MARAGazeboPluginRosPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

/*  impl_->timer_motor_state_ = impl_->ros_node_->create_wall_timer(
        10ms, std::bind(&MARAGazeboPluginRosPrivate::timer_motor_state_msgs, impl_.get()));
*/
  impl_->UpdateJointPIDs();
}

float * MARAGazeboPluginRosPrivate::getPIDValues(std::string joint_name)
{
  static float pid_values[7];

  if (joint_name.compare("motor1")==0){
    pid_values[0]=5000;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=20.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }else if (joint_name.compare("motor2")==0){
    pid_values[0]=10000;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=20.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }else if (joint_name.compare("motor3")==0){
    pid_values[0]=15000;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=20.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }else if (joint_name.compare("motor4")==0){
    pid_values[0]=500;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=10.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }else if (joint_name.compare("motor5")==0){
    pid_values[0]=500;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=10.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }else if (joint_name.compare("motor6")==0){
    pid_values[0]=5;  //_p  The proportional gain.
    pid_values[1]=0.0;  //_i  The integral gain.
    pid_values[2]=1.0;  //_d  The derivative gain.
    pid_values[3]=0.0;  //_imax The integral upper limit.
    pid_values[4]=0.0;  //_imin The integral lower limit.
    pid_values[5]=-3.1416;  //_cmdMax Output max value.
    pid_values[6]=3.1416;  //_cmdMin Output min value.
  }
  return pid_values;
}

void MARAGazeboPluginRos::Reset()
{
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
}

void MARAGazeboPluginRosPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  // TODO, this is a long OnUpdate, we will need to check the does not get overlapped by next.

  // AXIS 1
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

  // AXIS 2
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

  // AXIS 3
  if(!executing_axis3 && trajectories_position_axis3.size()>0){
    index_trajectory_axis3 = 0;
    executing_axis3 = true;
  }
  if(executing_axis3){
    goal_position_axis3_rad = trajectories_position_axis3[index_trajectory_axis3];
    index_trajectory_axis3++;
    if(index_trajectory_axis3==trajectories_position_axis3.size()){
      executing_axis3 = false;
      trajectories_position_axis3.clear();
      index_trajectory_axis3 = 0;
    }
  }

  // AXIS 4
  if(!executing_axis4 && trajectories_position_axis4.size()>0){
    index_trajectory_axis4 = 0;
    executing_axis4 = true;
  }
  if(executing_axis4){
    goal_position_axis4_rad = trajectories_position_axis4[index_trajectory_axis4];
    index_trajectory_axis4++;
    if(index_trajectory_axis4==trajectories_position_axis4.size()){
      executing_axis4 = false;
      trajectories_position_axis4.clear();
      index_trajectory_axis4 = 0;
    }
  }

  // AXIS 5
  if(!executing_axis5 && trajectories_position_axis5.size()>0){
    index_trajectory_axis5 = 0;
    executing_axis5 = true;
  }
  if(executing_axis5){
    goal_position_axis5_rad = trajectories_position_axis5[index_trajectory_axis5];
    index_trajectory_axis5++;
    if(index_trajectory_axis5==trajectories_position_axis5.size()){
      executing_axis5 = false;
      trajectories_position_axis5.clear();
      index_trajectory_axis5 = 0;
    }
  }

  // AXIS 6
  if(!executing_axis6 && trajectories_position_axis6.size()>0){
    index_trajectory_axis6 = 0;
    executing_axis6 = true;
  }
  if(executing_axis6){
    goal_position_axis6_rad = trajectories_position_axis6[index_trajectory_axis6];
    index_trajectory_axis6++;
    if(index_trajectory_axis6==trajectories_position_axis6.size()){
      executing_axis6 = false;
      trajectories_position_axis6.clear();
      index_trajectory_axis6 = 0;
    }
  }

  UpdatePIDControl();
  timer_motor_state_msgs();

  last_update_time_ = _info.simTime;
}

void MARAGazeboPluginRosPrivate::UpdateJointPIDs(){
  // AXIS 1
  float *motor1_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetName());
  float m1_p = *(motor1_pid + 0);
  float m1_i = *(motor1_pid + 1);
  float m1_d = *(motor1_pid + 2);
  float m1_imax = *(motor1_pid + 3);
  float m1_imin = *(motor1_pid + 4);
  //float m1_cmdMax = *(motor1_pid + 5);
  //float m1_cmdMin = *(motor1_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetScopedName(),
    gazebo::common::PID(m1_p, m1_i, m1_d, m1_imax, m1_imin, joints_[MARAGazeboPluginRosPrivate::AXIS1]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS1]->UpperLimit(0)));

  // AXIS 2
  float *motor2_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetName());
  float m2_p = *(motor2_pid + 0);
  float m2_i = *(motor2_pid + 1);
  float m2_d = *(motor2_pid + 2);
  float m2_imax = *(motor2_pid + 3);
  float m2_imin = *(motor2_pid + 4);
  //float m2_cmdMax = *(motor2_pid + 5);
  //float m2_cmdMin = *(motor2_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetScopedName(),
    gazebo::common::PID(m2_p, m2_i, m2_d, m2_imax, m2_imin, joints_[MARAGazeboPluginRosPrivate::AXIS2]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS2]->UpperLimit(0)));

  // AXIS 3
  float *motor3_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS3]->GetName());
  float m3_p = *(motor3_pid + 0);
  float m3_i = *(motor3_pid + 1);
  float m3_d = *(motor3_pid + 2);
  float m3_imax = *(motor3_pid + 3);
  float m3_imin = *(motor3_pid + 4);
  //float m3_cmdMax = *(motor3_pid + 5);
  //float m3_cmdMin = *(motor3_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS3]->GetScopedName(),
    gazebo::common::PID(m3_p, m3_i, m3_d, m3_imax, m3_imin, joints_[MARAGazeboPluginRosPrivate::AXIS3]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS3]->UpperLimit(0)));

  // AXIS 4
  float *motor4_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS4]->GetName());
  float m4_p = *(motor4_pid + 0);
  float m4_i = *(motor4_pid + 1);
  float m4_d = *(motor4_pid + 2);
  float m4_imax = *(motor4_pid + 3);
  float m4_imin = *(motor4_pid + 4);
  //float m4_cmdMax = *(motor4_pid + 5);
  //float m4_cmdMin = *(motor4_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS4]->GetScopedName(),
    gazebo::common::PID(m4_p, m4_i, m4_d, m4_imax, m4_imin, joints_[MARAGazeboPluginRosPrivate::AXIS4]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS4]->UpperLimit(0)));

  // AXIS 5
  float *motor5_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS5]->GetName());
  float m5_p = *(motor5_pid + 0);
  float m5_i = *(motor5_pid + 1);
  float m5_d = *(motor5_pid + 2);
  float m5_imax = *(motor5_pid + 3);
  float m5_imin = *(motor5_pid + 4);
  //float m5_cmdMax = *(motor5_pid + 5);
  //float m5_cmdMin = *(motor5_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS5]->GetScopedName(),
    gazebo::common::PID(m5_p, m5_i, m5_d, m5_imax, m5_imin, joints_[MARAGazeboPluginRosPrivate::AXIS5]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS5]->UpperLimit(0)));

  // AXIS 6
  float *motor6_pid = getPIDValues(joints_[MARAGazeboPluginRosPrivate::AXIS6]->GetName());
  float m6_p = *(motor6_pid + 0);
  float m6_i = *(motor6_pid + 1);
  float m6_d = *(motor6_pid + 2);
  float m6_imax = *(motor6_pid + 3);
  float m6_imin = *(motor6_pid + 4);
  //float m6_cmdMax = *(motor6_pid + 5);
  //float m6_cmdMin = *(motor6_pid + 6);

  model_->GetJointController()->SetPositionPID(
    joints_[MARAGazeboPluginRosPrivate::AXIS6]->GetScopedName(),
    gazebo::common::PID(m6_p, m6_i, m6_d, m6_imax, m6_imin, joints_[MARAGazeboPluginRosPrivate::AXIS6]->LowerLimit(0), joints_[MARAGazeboPluginRosPrivate::AXIS6]->UpperLimit(0)));

}

void MARAGazeboPluginRosPrivate::UpdatePIDControl()
{
  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS1]->GetScopedName(), goal_position_axis1_rad);

  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS2]->GetScopedName(), goal_position_axis2_rad);
  
  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS3]->GetScopedName(), goal_position_axis3_rad);

  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS4]->GetScopedName(), goal_position_axis4_rad);

  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS5]->GetScopedName(), goal_position_axis5_rad);

  model_->GetJointController()->SetPositionTarget(
    joints_[MARAGazeboPluginRosPrivate::AXIS6]->GetScopedName(), goal_position_axis6_rad);

}

GZ_REGISTER_MODEL_PLUGIN(MARAGazeboPluginRos)
}  // namespace gazebo_plugins
