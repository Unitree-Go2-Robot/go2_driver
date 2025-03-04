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


#ifndef GO2_DRIVER__GO2_JOINT_STATES_HPP
#define GO2_DRIVER__GO2_JOINT_STATES_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include "go2_driver/utils/go2_api_id.hpp"
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"
#include "unitree_go/msg/low_state.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2JointStates : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2JointStates(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2JointStates() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg);

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_JOINT_STATES_HPP
