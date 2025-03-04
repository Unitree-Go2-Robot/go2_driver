// Copyright 2025 Intelligent Robotics Lab
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


#include <go2_driver/modules/go2_joint_states.hpp>


namespace go2_driver
{

Go2JointStates::Go2JointStates(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node)
{
}

CallbackReturnT Go2JointStates::on_configure()
{
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mJoint states module configured.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2JointStates::on_activate()
{
  joint_state_pub_->on_activate();

  low_state_sub_ = node_->create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10,
    std::bind(&Go2JointStates::publish_joint_states, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mJoint states module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2JointStates::on_deactivate()
{
  joint_state_pub_->on_deactivate();

  low_state_sub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mJoint states module deactivated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2JointStates::on_cleanup()
{
  joint_state_pub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mJoint states module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2JointStates::publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->now();
  joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  joint_state.position = {msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
    msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
    msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q,
    msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q};

  joint_state_pub_->publish(joint_state);
}

}  // namespace go2_driver
