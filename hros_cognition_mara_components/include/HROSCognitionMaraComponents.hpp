#ifndef HROSCOGNITIONSCARACOMPONETSNODE_H
#define HROSCOGNITIONSCARACOMPONETSNODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/cmdline_parser.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

#include <yaml-cpp/yaml.h>


class HROSCognitionMaraComponentsNode : public rclcpp::Node
{
  public:
    /// LifecycleTalker constructor
    /**
     * The lifecycletalker/lifecyclenode constructor has the same
     * arguments a regular node.
     */
    explicit HROSCognitionMaraComponentsNode(const std::string & node_name,
                        int argc, char **argv);

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
    std::vector<std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>>> motor_goal_publishers_;

  	std::shared_ptr<rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>> common_joints_pub_;
    std::shared_ptr<rclcpp::TimerBase> timer_common_joints_;

    std::vector<std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>>> list_available_publishers;

  	std::shared_ptr<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>> trajectory_sub_;
    void commandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void stateCallback(std::string motor_name, float velocity, float position, float effort);

    std::vector<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo> cmd_to_send;

    pthread_mutex_t mutex_command;

    double nan;

    pthread_mutex_t mtx;
  	control_msgs::msg::JointTrajectoryControllerState msg_actuators_;

    std::string file_motors;
    std::string environment;
    std::string motor_key;

    std::vector<std::string> motor_names;
    std::vector<bool> msg_actuators_callback_sync;

};

#endif // HROSCOGNITIONSCARACOMPONETSNODE_H
