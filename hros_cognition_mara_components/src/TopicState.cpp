#include "hros_cognition_mara_components/HROSCognitionMaraComponents.hpp"

void HROSCognitionMaraComponentsNode::stateCallback(std::string motor_name, float velocity, float position, float effort)
{
  pthread_mutex_lock( &mtx );

  auto pos = std::find(msg_actuators_.joint_names.begin(), msg_actuators_.joint_names.end(), motor_name) - msg_actuators_.joint_names.begin();
  msg_actuators_.actual.positions[pos] = position;
  msg_actuators_.actual.velocities[pos] = velocity;
  msg_actuators_.actual.effort[pos] = effort;

  msg_actuators_callback_sync[pos] = true;

  // sync: check all elements are true
  if (std::find(msg_actuators_callback_sync.begin(), msg_actuators_callback_sync.end(), false) == msg_actuators_callback_sync.end()) {
    // publish synced messages
    timer_stateCommonPublisher();
    // reset sync tool
    for(unsigned int i = 0; i < msg_actuators_callback_sync.size(); i++){
      msg_actuators_callback_sync[i] =  false;
    }
  }

  pthread_mutex_unlock( &mtx );
}

void HROSCognitionMaraComponentsNode::timer_stateCommonPublisher()
{
  control_msgs::msg::JointTrajectoryControllerState msg_actuators;

  msg_actuators = msg_actuators_;
  for(unsigned int i = 0; i < msg_actuators.joint_names.size(); i++){
    msg_actuators.joint_names[i] = std::string("motor") + std::to_string(i+1);
  }

  builtin_interfaces::msg::Time stamp = clock_ros.now();
  msg_actuators.header.stamp.sec = stamp.sec;
  msg_actuators.header.stamp.nanosec = stamp.nanosec;

  common_joints_pub_->publish(msg_actuators);
}
