/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
    This file has been modified from the original, by Devon Ash
*/

#include <ros/ros.h>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
// #include <gazebo/math/Angle.hh>
#include <gazebo/physics/physics.hh>
#include <robotiq_arg2f_model_articulated_gazebo_plugins/Robotiq140Plugin.h>

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


  bool RobotiqHandPlugin::gripper_service(std_srvs::Empty::Request  &req,
           std_srvs::Empty::Response &res)
  {

    rosnode_->getParam( "kp_gripper", kp );
    rosnode_->getParam( "ki_gripper", ki );
    rosnode_->getParam( "kd_gripper", kd );

    rosnode_->getParam( "imin_gripper", imin );
    rosnode_->getParam( "imax_gripper", imax );

    rosnode_->getParam( "cmdmin_gripper", cmdmin );
    rosnode_->getParam( "cmdmax_gripper", cmdmax );

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

    gzerr << "Position PID parameters for joints "  << std::endl
          << "\tKP: "     << posePID_left_inner_knuckle.GetPGain()  << std::endl
          << "\tKI: "     << posePID_left_inner_knuckle.GetIGain()  << std::endl
          << "\tKD: "     << posePID_left_inner_knuckle.GetDGain()  << std::endl
          << "\tIMin: "   << posePID_left_inner_knuckle.GetIMin()   << std::endl
          << "\tIMax: "   << posePID_left_inner_knuckle.GetIMax()   << std::endl
          << "\tCmdMin: " << posePID_left_inner_knuckle.GetCmdMin() << std::endl
          << "\tCmdMax: " << posePID_left_inner_knuckle.GetCmdMax() << std::endl
          << std::endl;

    return true;
  }


  ////////////////////////////////////////////////////////////////////////////////
  void RobotiqHandPlugin::Load(gazebo::physics::ModelPtr _parent,
                               sdf::ElementPtr _sdf)
  {

    std::string robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
       robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    else{
      printf("No robotNamespace\n");
    }
    rosnode_ = new ros::NodeHandle();
    // Check that ROS has been initialized
    if(!ros::isInitialized()){
      ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
      return;
    }

    rosnode_->getParam( "kp_gripper", kp );
    rosnode_->getParam( "ki_gripper", ki );
    rosnode_->getParam( "kd_gripper", kd );

    rosnode_->getParam( "imin_gripper", imin );
    rosnode_->getParam( "imax_gripper", imax );

    rosnode_->getParam( "cmdmin_gripper", cmdmin );
    rosnode_->getParam( "cmdmax_gripper", cmdmax );

    printf("kp %.2f\n", kp);

    printf("RobotiqHandPlugin::Load\n");
    gzerr << "RobotiqHandPlugin::Load" << std::endl;
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->sdf = _sdf;

    // Error message if the model couldn't be found
    if (!this->model){
      gzerr<< "Parent model is NULL! RobotiqHandPlugin could not be loaded."<< std::endl;
      return;
    }
    gzerr<< "robot_namespace_ " << robot_namespace_ << std::endl;

    gzerr<< "_sdf description " << _sdf->GetDescription() << std::endl;

    for(int i = 0; i < model->GetJoints().size(); i++){
        gzerr << this->model->GetJoints()[i]->GetScopedName() << std::endl;
    }

    posePID_left_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_left_inner_knuckle.SetCmd(0.0);
    posePID_right_inner_knuckle.Init(kp, ki, kd, imax, imin, cmdmax, cmdmin);
    posePID_right_inner_knuckle.SetCmd(0.0);

    left_inner_knuckle_joint = this->model->GetJoint("left_inner_knuckle_joint");
    if (!left_inner_knuckle_joint){
      gzthrow("could not find front left_inner_knuckle_joint\n");
    }
    gzerr << "left_inner_knuckle_joint LowerLimit " << left_inner_knuckle_joint->LowerLimit(0) << std::endl;
    gzerr << "left_inner_knuckle_joint UpperLimit " << left_inner_knuckle_joint->UpperLimit(0) << std::endl;
    gzerr << "left_inner_knuckle_joint GetEffortLimit " << left_inner_knuckle_joint->GetEffortLimit(0) << std::endl;
    gzerr << "left_inner_knuckle_joint GetVelocityLimit " << left_inner_knuckle_joint->GetVelocityLimit(0) << std::endl;
    posePID_left_inner_knuckle.SetCmdMin(-left_inner_knuckle_joint->GetEffortLimit(0));
    posePID_left_inner_knuckle.SetCmdMax(left_inner_knuckle_joint->GetEffortLimit(0));

    right_inner_knuckle_joint = this->model->GetJoint("right_inner_knuckle_joint");
    if (!right_inner_knuckle_joint){
      gzthrow("could not find front right_inner_knuckle_joint\n");
    }
    gzerr << "right_inner_knuckle_joint LowerLimit " << right_inner_knuckle_joint->LowerLimit(0) << std::endl;
    gzerr << "right_inner_knuckle_joint UpperLimit " << right_inner_knuckle_joint->UpperLimit(0) << std::endl;
    gzerr << "right_inner_knuckle_joint GetEffortLimit " << right_inner_knuckle_joint->GetEffortLimit(0) << std::endl;
    gzerr << "right_inner_knuckle_joint GetVelocityLimit " << right_inner_knuckle_joint->GetVelocityLimit(0) << std::endl;
    posePID_right_inner_knuckle.SetCmdMin(-right_inner_knuckle_joint->GetEffortLimit(0));
    posePID_right_inner_knuckle.SetCmdMax(right_inner_knuckle_joint->GetEffortLimit(0));

    this->lastControllerUpdateTime = this->world->SimTime();

    ros::AdvertiseServiceOptions aso1 =
                ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "open_gripper", boost::bind(&RobotiqHandPlugin::gripper_service,
                this, _1, _2), ros::VoidPtr(), &queue_);

    service1 = rosnode_->advertiseService(aso1);
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &RobotiqHandPlugin::QueueThread, this ) );

    // Connect to gazebo world update.
    this->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotiqHandPlugin::UpdateStates, this));

    sentido = -1;

  }

  void RobotiqHandPlugin::QueueThread()
  {
    double timeout = 0.01;

    while ( rosnode_->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
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

    // gzerr << "targetPose: "  << targetPose << " currentPose: " << currentPose << std::endl;

    // Position error.
    double poseError_right = currentPose_right - targetPose_right;
    double poseError_left = currentPose_left - targetPose_left;

    // Update the PID.
    double torque_right = posePID_right_inner_knuckle.Update(poseError_right, _dt);
    double torque_left = posePID_left_inner_knuckle.Update(poseError_left, _dt);

    // gzerr << "torque_right: "  << torque_right << " poseError_right: " << poseError_right << std::endl;
    // gzerr << "torque_left: "  << torque_left << " poseError_left: " << poseError_left << std::endl;

    // Apply the PID command.
    right_inner_knuckle_joint->SetForce(0, torque_right);
    left_inner_knuckle_joint->SetForce(0, torque_left);
  }

  GZ_REGISTER_MODEL_PLUGIN(RobotiqHandPlugin)
}
