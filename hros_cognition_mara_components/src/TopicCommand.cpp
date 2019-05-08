#include "HROSCognitionMaraComponents.hpp"

void HROSCognitionMaraComponentsNode::commandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  pthread_mutex_lock( &mutex_command );
  cmd_to_send.clear();

  for(unsigned int i = 0; i < msg->points[0].positions.size(); i++){
    hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo cmd_msg1;
    cmd_msg1.position = msg->points[0].positions[i];
    if(msg->points[0].velocities.size() > i){
      cmd_msg1.velocity = abs(msg->points[0].velocities[i]);
    }else{
      cmd_msg1.velocity = 0.0;
      RCUTILS_LOG_INFO_NAMED(get_name(), "HROSCognitionMaraComponentsNode::commandCallback() you are not defining the velocities!!.");
    }
    cmd_msg1.effort = 0;
    cmd_msg1.control_type = hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::CONTROL_TYPE_POSITION_VELOCITY;
    cmd_msg1.header.frame_id = std::string("None");

    builtin_interfaces::msg::Time stamp = clock_ros.now();
    cmd_msg1.header.stamp.sec = stamp.sec;
    cmd_msg1.header.stamp.nanosec = stamp.nanosec;

    cmd_to_send.push_back(cmd_msg1);
  }

  timer_commandPublisher();

  pthread_mutex_unlock( &mutex_command );
}

void HROSCognitionMaraComponentsNode::timer_commandPublisher()
{
  msg_actuators_.error.positions.clear();
  if(cmd_to_send.size()>0){
    for(unsigned int i = 0; i < motor_goal_publishers_.size(); i++ ){

      hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo cmd_msg = cmd_to_send.front();
      cmd_to_send.erase (cmd_to_send.begin());
      msg_actuators_.error.positions.push_back(msg_actuators_.actual.positions[i] - cmd_msg.position);
      cmd_msg.control_type = hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::CONTROL_TYPE_POSITION_VELOCITY;
      motor_goal_publishers_[i]->publish(cmd_msg);
    }
  }
}
