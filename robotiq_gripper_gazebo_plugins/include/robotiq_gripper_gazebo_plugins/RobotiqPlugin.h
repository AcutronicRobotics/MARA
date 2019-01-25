/*
 * Copyright 2014 Open Source Robotics Foundation
 * Copyright 2015 Clearpath Robotics
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

#ifndef GAZEBO_ROBOTIQ_HAND_PLUGIN_HH
#define GAZEBO_ROBOTIQ_HAND_PLUGIN_HH

#include <string>
#include <vector>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <hrim_actuator_gripper_msgs/msg/state_gripper.hpp>
#include <hrim_actuator_gripper_msgs/msg/state_finger_gripper.hpp>
#include <hrim_actuator_gripper_msgs/msg/specs_finger_gripper.hpp>
#include <hrim_actuator_gripper_srvs/srv/control_finger.hpp>

// HRIM messages
#include <hrim_generic_msgs/msg/id.hpp>
#include <hrim_generic_msgs/msg/status.hpp>
#include <hrim_generic_msgs/msg/power.hpp>
#include <hrim_generic_msgs/msg/simulation3_d.hpp>
#include <hrim_generic_msgs/msg/simulation_urdf.hpp>
#include <hrim_generic_msgs/msg/specs_communication.hpp>
#include <hrim_generic_msgs/msg/state_communication.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#define MIN_FORCE 10
#define MAX_FORCE 125

#define MAX_PAYLOAD 2.5

#define MIN_SPEED 30
#define MAX_SPEED 250

#define MAX_ACCELERATION 0

#define MAX_LENGHT 209
#define MAX_ANGLE 0.87

#define REPEATABILITY 0.08

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std::chrono_literals;

/// \brief A plugin that implements the Robotiq 2-Finger Adaptative Gripper.
namespace gazebo
  {
  class RobotiqHandPlugin : public gazebo::ModelPlugin
  {

    void gripper_service(const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Request> request,
          std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlFinger::Response> response);

    void createGenericTopics(std::string node_name);

    /// \brief Constructor.
    public: RobotiqHandPlugin();

    /// \brief Destructor.
    public: virtual ~RobotiqHandPlugin();

    private: uint8_t GetCurrentPosition(const gazebo::physics::JointPtr &_joint);
    private: bool IsHandFullyOpen();

    private: void UpdatePIDControl(double _dt);

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    public: void UpdateStates();

    /// \brief World pointer.
    private: gazebo::physics::WorldPtr world;

    /// \brief Parent model of the hand.
    private: gazebo::physics::ModelPtr model;

    /// \brief Pointer to the SDF of this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief gazebo world update connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    private: std::vector<gazebo::common::PID> left_posePID_v_;
    private: std::vector<gazebo::common::PID> right_posePID_v_;
    private: std::vector<physics::JointPtr> left_joint_v_;
    private: std::vector<physics::JointPtr> right_joint_v_;

    private: gazebo::common::Time lastControllerUpdateTime;

    /// \brief Min. joint speed (rad/s). Finger is 125mm and tip speed is 22mm/s.
    private: static const double MinVelocity = 0.176;

    /// \brief Max. joint speed (rad/s). Finger is 125mm and tip speed is 110mm/s.
    private: static const double MaxVelocity = 0.88;

    /// \brief Velocity tolerance. Below this value we assume that the joint is
    /// stopped (rad/s).
    private: static const double VelTolerance = 0.002;

    /// \brief Position tolerance. If the difference between target position and
    /// current position is within this value we'll conclude that the joint
    /// reached its target (rad).
    private: static const double PoseTolerance = 0.002;

    int sentido = 1;
    int count = 0;

    double kp = 5.0;
    double ki = 0.2;
    double kd = 0.05;
    double imin = 0.0;
    double imax = 0.0;
    double cmdmax = 10.0;
    double cmdmin = -10.0;

    double targetPose_right = 0.0;
    double targetPose_left  = 0.0;

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Service<hrim_actuator_gripper_srvs::srv::ControlFinger>::SharedPtr srv_;

    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::ID>> info_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::Status>> status_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::Power>> power_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::Simulation3D>> sim3d_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::SimulationURDF>> sim_urdf_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::StateCommunication>> state_comm_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_generic_msgs::msg::SpecsCommunication>> specs_comm_pub;

    std::shared_ptr<rclcpp::Publisher<hrim_actuator_gripper_msgs::msg::StateFingerGripper>> gripper_finger_state_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_gripper_msgs::msg::SpecsFingerGripper>> specs_pub;
    std::shared_ptr<rclcpp::Publisher<hrim_actuator_gripper_msgs::msg::StateGripper>> gripper_state_pub;

    void timer_info_msgs();
    void timer_power_msgs();
    void timer_status_msgs();
    void timer_specs_msgs();
    void timer_comm_msgs();
    void timer_gripper_status_msgs();

    void publish3DModels();
    void readfullFile(std::string file_to_read, hrim_generic_msgs::msg::Simulation3D& msg_sim_3d);

    std::shared_ptr<rclcpp::TimerBase> timer_info_;
    std::shared_ptr<rclcpp::TimerBase> timer_status_;
    std::shared_ptr<rclcpp::TimerBase> timer_gripper_status_;
    std::shared_ptr<rclcpp::TimerBase> timer_power_;
    std::shared_ptr<rclcpp::TimerBase> timer_specs_;
    std::shared_ptr<rclcpp::TimerBase> timer_comm_;

  };
}
#endif  // GAZEBO_ROBOTIQ_HAND_PLUGIN_HH
