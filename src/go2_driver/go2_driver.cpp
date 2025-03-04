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

#include <go2_driver/go2_driver.hpp>


namespace go2_driver
{

Go2Driver::Go2Driver(
  const rclcpp::NodeOptions & options)
: LifecycleNode("go2_driver", options)
{
  declare_parameter("use_camera", true);
  declare_parameter("use_tts", true);
  declare_parameter("use_vui", true);
  declare_parameter("use_odometry", true);
  declare_parameter("use_joint_states", true);
  declare_parameter("use_services", true);

  get_parameter("use_camera", use_camera_);
  get_parameter("use_tts", use_tts_);
  get_parameter("use_vui", use_vui_);
  get_parameter("use_odometry", use_odometry_);
  get_parameter("use_joint_states", use_joint_states_);
  get_parameter("use_services", use_services_);
}

CallbackReturnT Go2Driver::on_configure(const rclcpp_lifecycle::State &)
{
  if (use_camera_) {
    go2_camera_ = std::make_shared<go2_driver::Go2Camera>(shared_from_this());
    go2_camera_->on_configure();
  }

  if (use_tts_) {
    go2_tts_ = std::make_shared<go2_driver::Go2TTS>(shared_from_this());
    go2_tts_->on_configure();
  }

  if (use_vui_) {
    go2_vui_ = std::make_shared<go2_driver::Go2VUI>(shared_from_this());
    go2_vui_->on_configure();
  }

  if (use_odometry_) {
    go2_odometry_ = std::make_shared<go2_driver::Go2Odometry>(shared_from_this());
    go2_odometry_->on_configure();
  }

  if (use_joint_states_) {
    go2_joint_states_ = std::make_shared<go2_driver::Go2JointStates>(shared_from_this());
    go2_joint_states_->on_configure();
  }

  if (use_services_) {
    go2_services_handler_ = std::make_shared<go2_driver::Go2HandleServices>(shared_from_this());
    go2_services_handler_->on_configure();
  }

  RCLCPP_INFO(get_logger(), "\033[1;32mAll modules configured\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Driver::on_activate(const rclcpp_lifecycle::State &)
{
  if (use_camera_) {
    go2_camera_->on_activate();
  }

  if (use_tts_) {
    go2_tts_->on_activate();
  }

  if (use_vui_) {
    go2_vui_->on_activate();
  }

  if (use_odometry_) {
    go2_odometry_->on_activate();
  }

  if (use_joint_states_) {
    go2_joint_states_->on_activate();
  }

  if (use_services_) {
    go2_services_handler_->on_activate();
  }

  RCLCPP_INFO(get_logger(), "\033[1;32mAll modules activated\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (use_camera_) {
    go2_camera_->on_deactivate();
  }

  if (use_tts_) {
    go2_tts_->on_deactivate();
  }

  if (use_vui_) {
    go2_vui_->on_deactivate();
  }

  if (use_odometry_) {
    go2_odometry_->on_deactivate();
  }

  if (use_joint_states_) {
    go2_joint_states_->on_deactivate();
  }

  if (use_services_) {
    go2_services_handler_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "\033[1;32mAll modules deactivated\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (use_camera_) {
    go2_camera_->on_cleanup();
    go2_camera_.reset();
  }

  if (use_tts_) {
    go2_tts_->on_cleanup();
    go2_tts_.reset();
  }

  if (use_vui_) {
    go2_vui_->on_cleanup();
    go2_vui_.reset();
  }

  if (use_odometry_) {
    go2_odometry_->on_cleanup();
    go2_odometry_.reset();
  }

  if (use_joint_states_) {
    go2_joint_states_->on_cleanup();
    go2_joint_states_.reset();
  }

  if (use_services_) {
    go2_services_handler_->on_cleanup();
    go2_services_handler_.reset();
  }

  RCLCPP_INFO(get_logger(), "\033[1;32mAll modules cleaned up\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturnT::SUCCESS;
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2Driver)
