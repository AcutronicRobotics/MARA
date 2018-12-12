#ifndef HROSCOGNITIONSCARACOMPONETSNODE_H
#define HROSCOGNITIONSCARACOMPONETSNODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/specs_rotary_servo.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/cmdline_parser.h"

// hebi api includes
#include "hebi/lookup.hpp"
#include "hebi/group.hpp"
#include "hebi/group_command.hpp"
#include "hebi/trajectory.hpp"
#include "hebi/kinematics.hpp"
#include "Eigen/Eigen"

using namespace std::chrono_literals;
using namespace std::placeholders;

#include <yaml-cpp/yaml.h>

/// LifeycycleTalker inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * The lifecycle talker does not like the regular "talker" node
 * inherit from node, but rather from lifecyclenode. This brings
 * in a set of callbacks which are getting invoked depending on
 * the current state of the node.
 * Every lifecycle node has a set of services attached to it
 * which make it controllable from the outside and invoke state
 * changes.
 * Available Services as for Beta1:
 * - <node_name>__get_state
 * - <node_name>__change_state
 * - <node_name>__get_available_states
 * - <node_name>__get_available_transitions
 * Additionally, a publisher for state change notifications is
 * created:
 * - <node_name>__transition_event
 */

class HROSCognitionMaraComponentsNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  /// LifecycleTalker constructor
  /**
   * The lifecycletalker/lifecyclenode constructor has the same
   * arguments a regular node.
   */
  explicit HROSCognitionMaraComponentsNode(const std::string & node_name,
                      int argc, char **argv, bool intra_process_comms = false);

  void init_state_machine();

  /// Callback for walltimer in order to publish the message.
  /**
   * Callback for walltimer. This function gets invoked by the timer
   * and executes the publishing.
   * For this demo, we ask the node for its current state. If the
   * lifecycle publisher is not activate, we still invoke publish, but
   * the communication is blocked so that no messages is actually transferred.
   */
   void timer_stateCommonPublisher();
   void timer_commandPublisher();

   std::thread* the_thread;
   std::vector<std::string> node_names_;
   std::mutex mutex_node_names;

  /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  /// Transition callback for state activating
 /**
  * on_activate callback is being called when the lifecycle node
  * enters the "activating" state.
  * Depending on the return value of this function, the state machine
  * either invokes a transition to the "active" state or stays
  * in "inactive".
  * TRANSITION_CALLBACK_SUCCESS transitions to "active"
  * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
  * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

   /// Transition callback for state activating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
   * TRANSITION_CALLBACK_FAILURE transitions to "active"
   * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

    /// Transition callback for state deactivating
    /**
     * on_cleanup callback is being called when the lifecycle node
     * enters the "cleaningup" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "uncofigured" state or stays
     * in "inactive".
     * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
     * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
     */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &);

private:
  rclcpp::Clock clock_ros;
  // We hold an instance of a lifecycle publisher. This lifecycle publisher
  // can be activated or deactivated regarding on which state the lifecycle node
  // is in.

  // We hold an instance of a timer which periodically triggers the publish function.
  // As for the beta version, this is a regular timer. In a future version, a
  // lifecycle timer will be created which obeys the same lifecycle management as the
  // lifecycle publisher.
  std::shared_ptr<rclcpp::TimerBase> timer_command_;

  std::string node_name;

  std::vector<std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>>> motor_state_subcriptions_;
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>>> motor_goal_publishers_;

	std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointTrajectoryControllerState>> common_joints_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_common_joints_;

  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>>> list_available_publishers;

	std::shared_ptr<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>> trajectory_sub_;
  void commandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void stateCallback(std::string motor_name, float velocity, float position, float effort);

  std::vector<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo> cmd_to_send;

  pthread_mutex_t mutex_command;

  double nan;

  pthread_mutex_t mtx;
	control_msgs::msg::JointTrajectoryControllerState msg_actuators_;

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

  std::string file_motors;

};

#endif // HROSCOGNITIONSCARACOMPONETSNODE_H
