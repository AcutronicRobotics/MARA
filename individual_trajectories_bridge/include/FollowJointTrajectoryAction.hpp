#ifndef FOLLOWJOINTTRAJECTORYACTION_H
#define FOLLOWJOINTTRAJECTORYACTION_H

#include <iostream>
#include <yaml-cpp/yaml.h>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib/server/simple_action_server.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rcutils/cmdline_parser.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "hrim_actuator_rotaryservo_actions/action/goal_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <unistd.h>
#include <cstring>


using namespace std::chrono_literals;

class FollowJointTrajectoryAction
{
public:

  FollowJointTrajectoryAction(std::string name, ros::NodeHandle nh, rclcpp::Node::SharedPtr node_ros2);

  ~FollowJointTrajectoryAction();

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr,
    const std::shared_ptr<const hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Feedback> feedback);

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  void actionCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

protected:

  rclcpp::Node::SharedPtr node_ros2;
  ros::Subscriber sub_action_cancel_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr goal_handle;

  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryResult result_;

  rclcpp_action::Client<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr action_client;

};
#endif // FOLLOWJOINTTRAJECTORYACTION_H
