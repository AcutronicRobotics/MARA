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
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request_header;

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
    posePID_left_inner_knuckle.SetCmd(0.0);
    posePID_right_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_right_inner_knuckle.SetCmd(0.0);

    left_inner_knuckle_joint = this->model->GetJoint("left_inner_knuckle_joint");
    if (!left_inner_knuckle_joint){
      gzthrow("could not find front left_inner_knuckle_joint\n");
    }
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
    gzmsg << "right_inner_knuckle_joint LowerLimit " << right_inner_knuckle_joint->LowerLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint UpperLimit " << right_inner_knuckle_joint->UpperLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint GetEffortLimit " << right_inner_knuckle_joint->GetEffortLimit(0) << std::endl;
    gzmsg << "right_inner_knuckle_joint GetVelocityLimit " << right_inner_knuckle_joint->GetVelocityLimit(0) << std::endl;
    posePID_right_inner_knuckle.SetCmdMin(-right_inner_knuckle_joint->GetEffortLimit(0));
    posePID_right_inner_knuckle.SetCmdMax(right_inner_knuckle_joint->GetEffortLimit(0));

    this->lastControllerUpdateTime = this->world->SimTime();

    std::function<void( std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>)> cb_fingercontrol_function = std::bind(
          &RobotiqHandPlugin::gripper_service, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

    srv_ = ros_node_->create_service<std_srvs::srv::Empty>("open_gripper", cb_fingercontrol_function);

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

    double targetPose_right = 0.0;
    double targetPose_left  = 0.0;

    if (sentido==1){
      targetPose_right = right_inner_knuckle_joint->UpperLimit(0);
      targetPose_left = left_inner_knuckle_joint->UpperLimit(0);
    }else{
      targetPose_right = right_inner_knuckle_joint->LowerLimit(0);
      targetPose_left = left_inner_knuckle_joint->LowerLimit(0);
    }

    // Get the current pose.
    double currentPose_right = right_inner_knuckle_joint->Position(0);
    double currentPose_left = left_inner_knuckle_joint->Position(0);

    // gzmsg << "targetPose: "  << targetPose << " currentPose: " << currentPose << std::endl;

    // Position error.
    double poseError_right = currentPose_right - targetPose_right;
    double poseError_left = currentPose_left - targetPose_left;

    // Update the PID.
    double torque_right = posePID_right_inner_knuckle.Update(poseError_right, _dt);
    double torque_left = posePID_left_inner_knuckle.Update(poseError_left, _dt);

    // gzmsg << "torque_right: "  << torque_right << " poseError_right: " << poseError_right << std::endl;
    // gzmsg << "torque_left: "  << torque_left << " poseError_left: " << poseError_left << std::endl;

    // Apply the PID command.
    right_inner_knuckle_joint->SetForce(0, torque_right);
    left_inner_knuckle_joint->SetForce(0, torque_left);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqHandPlugin)
}
