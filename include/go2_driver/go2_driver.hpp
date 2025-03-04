// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GO2_DRIVER__GO2_DRIVER_HPP_
#define GO2_DRIVER__GO2_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "go2_driver/modules/go2_camera.hpp"
#include "go2_driver/modules/go2_handle_services.hpp"
#include "go2_driver/modules/go2_tts.hpp"
#include "go2_driver/modules/go2_vui.hpp"
#include "go2_driver/modules/go2_odometry.hpp"
#include "go2_driver/modules/go2_joint_states.hpp"
#include "go2_driver/modules/go2_switch_obstacles_avoidance.hpp"


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  Go2Driver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  std::shared_ptr<go2_driver::Go2Camera> go2_camera_;
  std::shared_ptr<go2_driver::Go2HandleServices> go2_services_handler_;
  std::shared_ptr<go2_driver::Go2TTS> go2_tts_;
  std::shared_ptr<go2_driver::Go2VUI> go2_vui_;
  std::shared_ptr<go2_driver::Go2Odometry> go2_odometry_;
  std::shared_ptr<go2_driver::Go2JointStates> go2_joint_states_;
  std::shared_ptr<go2_driver::Go2SwitchObstaclesAvoidance> go2_switch_obstacles_avoidance_;

  bool use_camera_;
  bool use_tts_;
  bool use_vui_;
  bool use_odometry_;
  bool use_joint_states_;
  bool use_services_;
  bool use_switch_obstacles_avoidance_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_DRIVER_HPP_
