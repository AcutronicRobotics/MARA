#include "hros_cognition_mara_components/HROSCognitionMaraComponents.hpp"

void HROSCognitionMaraComponentsNode::stateCallback(std::string motor_name, float velocity, float position, float effort)
{
  pthread_mutex_lock( &mtx );

  auto pos = std::find(msg_actuators_.joint_names.begin(), msg_actuators_.joint_names.end(), motor_name) - msg_actuators_.joint_names.begin();
  msg_actuators_.actual.positions[pos] = position;
  msg_actuators_.actual.velocities[pos] = velocity;
  msg_actuators_.actual.effort[pos] = effort;

  timer_stateCommonPublisher();

  pthread_mutex_unlock( &mtx );


}

void HROSCognitionMaraComponentsNode::timer_stateCommonPublisher()
{
  control_msgs::msg::JointTrajectoryControllerState msg_actuators;

  //pthread_mutex_lock( &mtx );
  msg_actuators = msg_actuators_;
  for(unsigned int i = 0; i < msg_actuators.joint_names.size(); i++){
    msg_actuators.joint_names[i] = std::string("motor") + std::to_string(i+1);
  }

  builtin_interfaces::msg::Time stamp = clock_ros.now();
  msg_actuators.header.stamp.sec = stamp.sec;
  msg_actuators.header.stamp.nanosec = stamp.nanosec;

  common_joints_pub_->publish(msg_actuators);

  //pthread_mutex_unlock( &mtx );

}
