#include "FollowJointTrajectoryAction.hpp"

FollowJointTrajectoryAction::FollowJointTrajectoryAction(std::string name, ros::NodeHandle nh, rclcpp::Node::SharedPtr node_ros2) : as_(nh, name, boost::bind(&FollowJointTrajectoryAction::executeCB, this, _1), false), action_name_(name)
{
  as_.start();
  this->node_ros2 = node_ros2;
  action_client = rclcpp_action::create_client<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>(node_ros2, action_name_);

  sub_action_cancel_ = nh.subscribe( action_name_ + std::string("/cancel"), 1, &FollowJointTrajectoryAction::actionCancelCallback, this);
}

void FollowJointTrajectoryAction::actionCancelCallback(const actionlib_msgs::GoalID::ConstPtr&)
{

  auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
  if (cancel_result_future.wait_for(std::chrono::seconds(5s)) != std::future_status::ready)
  {
    printf("cancel goal call failed :(\n");
  }else{
    printf("cancel goal call ok! :)\n");
  }
}

FollowJointTrajectoryAction::~FollowJointTrajectoryAction()
{

}

void FollowJointTrajectoryAction::feedback_callback(
  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr,
  const std::shared_ptr<const hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Feedback> feedback)
{
  control_msgs::FollowJointTrajectoryFeedback feedback_;

  if(feedback_.actual.positions.size()>0)
    feedback_.actual.positions.push_back(feedback->actual.positions[0]);
  if(feedback_.actual.velocities.size()>0)
    feedback_.actual.velocities.push_back(feedback->actual.velocities[0]);
  feedback_.actual.time_from_start.sec = feedback->actual.time_from_start.sec;
  feedback_.actual.time_from_start.nsec = feedback->actual.time_from_start.nanosec;

  if(feedback_.desired.positions.size()>0)
    feedback_.desired.positions.push_back(feedback->desired.positions[0]);
  if(feedback_.desired.velocities.size()>0)
    feedback_.desired.velocities.push_back(feedback->desired.velocities[0]);
  feedback_.desired.time_from_start.sec = feedback->desired.time_from_start.sec;
  feedback_.desired.time_from_start.nsec = feedback->desired.time_from_start.nanosec;

  if(feedback_.error.positions.size()>0)
    feedback_.error.positions.push_back(feedback->error.positions[0]);
  if(feedback_.error.velocities.size()>0)
    feedback_.error.velocities.push_back(feedback->error.velocities[0]);
  feedback_.error.time_from_start.sec = feedback->error.time_from_start.sec;
  feedback_.error.time_from_start.nsec = feedback->error.time_from_start.nanosec;

  // re-publish the feedback
  as_.publishFeedback(feedback_);
}

void FollowJointTrajectoryAction::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  if (!action_client->wait_for_action_server(std::chrono::seconds(2))) {
    printf("Action server not available after waiting\n");
    return;
  }

  // Populate a goal
  auto goal_msg = hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal();

  goal_msg.trajectory.points.resize(goal->trajectory.points.size());
  for (unsigned int i = 0; i < goal->trajectory.points.size(); i++){
    goal_msg.trajectory.points[i].positions.resize(goal->trajectory.points[i].positions.size());
    for (unsigned int j = 0; j < goal->trajectory.points[i].positions.size(); j++ )
      goal_msg.trajectory.points[i].positions[j] = goal->trajectory.points[i].positions[j];

    goal_msg.trajectory.points[i].velocities.resize(goal->trajectory.points[i].velocities.size());
    for (unsigned int j = 0; j < goal->trajectory.points[i].velocities.size(); j++ )
      goal_msg.trajectory.points[i].velocities[j] = goal->trajectory.points[i].velocities[j];

    goal_msg.trajectory.points[i].accelerations.resize(goal->trajectory.points[i].accelerations.size());
    for (unsigned int j = 0; j < goal->trajectory.points[i].accelerations.size(); j++ )
      goal_msg.trajectory.points[i].accelerations[j] = goal->trajectory.points[i].accelerations[j];

    goal_msg.trajectory.points[i].time_from_start.sec = goal->trajectory.points[i].time_from_start.sec;
    goal_msg.trajectory.points[i].time_from_start.nanosec = goal->trajectory.points[i].time_from_start.nsec;
  }

  double wait_time = (double)(goal_msg.trajectory.points[goal->trajectory.points.size()-1].time_from_start.sec) +
                             (double)(goal_msg.trajectory.points[goal->trajectory.points.size()-1].time_from_start.nanosec/1e+9) + 1.0;


  printf("Sending goal\n");
  // std::function<void( rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr,
  //                     const std::shared_ptr<const hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Feedback> feedback )> cb_function = std::bind(
  //       &FollowJointTrajectoryAction::feedback_callback, this, std::placeholders::_1,  std::placeholders::_2);
  bool goal_response_received = false;
  auto send_goal_ops = rclcpp_action::Client<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SendGoalOptions();
  send_goal_ops.goal_response_callback =
    [&goal_response_received]
      (std::shared_future<typename rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::SharedPtr> future) mutable
    {
      auto goal_handle = future.get();
      if (goal_handle) {
        goal_response_received = true;
      }
    };

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_ops);

  if (goal_handle_future.wait_for(std::chrono::seconds(5s)) != std::future_status::ready)
  {
    printf("send goal call failed :(\n");
    return;
  }

  goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    printf("Goal was rejected by server\n");
    return;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(node_ros2->get_logger(), "Waiting for result %d seconds", ((int)wait_time) + 1);

  if (result_future.wait_for(std::chrono::seconds(((int)wait_time) + 1 )) != std::future_status::ready){
    RCLCPP_ERROR(node_ros2->get_logger(), "get result call failed :(");
    return;
  }

  rclcpp_action::ClientGoalHandle<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory>::WrappedResult result = result_future.get();

  switch(result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_ros2->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_ros2->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_ros2->get_logger(), "Unknown result code");
      return;
  }

  result_.error_code = 0;
  as_.setSucceeded(result_);
}
