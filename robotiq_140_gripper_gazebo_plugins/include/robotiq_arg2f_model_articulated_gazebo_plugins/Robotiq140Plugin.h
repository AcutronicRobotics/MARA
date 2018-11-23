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

#include <gazebo_plugins/PubQueue.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
// Gazebo
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <std_srvs/Empty.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

/// \brief A plugin that implements the Robotiq 2-Finger Adaptative Gripper.
namespace gazebo
  {
  class RobotiqHandPlugin : public gazebo::ModelPlugin
  {

    bool gripper_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
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

    private: ros::NodeHandle* rosnode_;

    /// \brief World pointer.
    private: gazebo::physics::WorldPtr world;

    /// \brief Parent model of the hand.
    private: gazebo::physics::ModelPtr model;

    /// \brief Pointer to the SDF of this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief gazebo world update connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    private: gazebo::common::PID posePID_left_inner_knuckle;
    private: gazebo::common::PID posePID_right_inner_knuckle;

    private: physics::JointPtr left_inner_knuckle_joint;
    private: physics::JointPtr right_inner_knuckle_joint;

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

    boost::thread callback_queue_thread_;
    ros::CallbackQueue queue_;
    void QueueThread();

    double kp = 20.0;
    double ki = 5.0;
    double kd = 0.2;
    double imin = 0.0;
    double imax = 0.0;
    double cmdmax = 10.0;
    double cmdmin = -10.0;
    ros::ServiceServer service1;
  };
}
#endif  // GAZEBO_ROBOTIQ_HAND_PLUGIN_HH
