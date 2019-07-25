// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_PLUGINS__MARA_GAZEBO_PLUGINS_HPP_
#define GAZEBO_PLUGINS__MARA_GAZEBO_PLUGINSE_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/state_rotary_servo.hpp"
#include "hrim_actuator_rotaryservo_msgs/msg/goal_rotary_servo.hpp"

#include "hrim_generic_msgs/msg/state_communication.hpp"

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mara_gazebo_plugins/spline.hpp>

static const double NSEC_PER_SECOND = 1e+9;

using namespace std::chrono_literals;

namespace gazebo_plugins
{
  class MARAGazeboPluginRosPrivate
  {
  public:

    /// Indicates which axis
    enum
    {
      AXIS1 = 0,
      AXIS2 = 1,
      AXIS3 = 2,
      AXIS4 = 3,
      AXIS5 = 4,
      AXIS6 = 5,
    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate();

    void timer_motor_state_msgs();
    std::shared_ptr<rclcpp::TimerBase> timer_motor_state_;

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Pointers to wheel joints.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    /// Robot base frame ID
    std::string robot_base_frame_;


    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis1_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis2_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis3_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis4_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis5_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_rotaryservo_msgs::msg::StateRotaryServo>> motor_state_axis6_pub;

    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis1_;
    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis2_;
    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis3_;
    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis4_;
    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis5_;
    std::shared_ptr<rclcpp::Subscription<hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo>> command_sub_axis6_;

    void commandCallback_axis1(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    void commandCallback_axis2(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    void commandCallback_axis3(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    void commandCallback_axis4(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    void commandCallback_axis5(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);
    void commandCallback_axis6(const hrim_actuator_rotaryservo_msgs::msg::GoalRotaryServo::SharedPtr msg);

    float goal_position_axis1_rad;
    float goal_position_axis2_rad;
    float goal_position_axis3_rad;
    float goal_position_axis4_rad;
    float goal_position_axis5_rad;
    float goal_position_axis6_rad;
  };

/// A plugin for gazebo.
/*
 *
 * \author  Alejandro Hernandez (alex <at> erlerobotics.com)
 * \author  Nestor Gonzalez (nestor <at> erlerobotics.com)
 * \author  Risto Kojcev (risto <at> erlerobotics.com)
 */

class MARAGazeboPluginRos : public gazebo::ModelPlugin
{
public:
  /// Constructor
  MARAGazeboPluginRos();

  /// Destructor
  ~MARAGazeboPluginRos();

  void createGenericTopics(std::string node_name);

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<MARAGazeboPluginRosPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__MARA_GAZEBO_PLUGINSE_HPP_