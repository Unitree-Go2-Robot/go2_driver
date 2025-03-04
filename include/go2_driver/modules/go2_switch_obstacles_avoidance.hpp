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


#ifndef GO2_DRIVER__GO2_SWITCH_OBSTACLES_AVOIDANCE_HPP
#define GO2_DRIVER__GO2_SWITCH_OBSTACLES_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "go2_interfaces/srv/get_switch_obstacles_avoidance.hpp"
#include "go2_interfaces/srv/set_switch_obstacles_avoidance.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include "go2_driver/utils/go2_api_id.hpp"

#include "nlohmann/json.hpp"


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2SwitchObstaclesAvoidance : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2SwitchObstaclesAvoidance(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2SwitchObstaclesAvoidance() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void handleGetObstaclesAvoidance(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetSwitchObstaclesAvoidance::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetSwitchObstaclesAvoidance::Response> response);

  void handleSetObstaclesAvoidance(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SetSwitchObstaclesAvoidance::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SetSwitchObstaclesAvoidance::Response> response);

  rclcpp::Service<go2_interfaces::srv::GetSwitchObstaclesAvoidance>::SharedPtr
    get_obstacles_avoidance_service_;
  rclcpp::Service<go2_interfaces::srv::SetSwitchObstaclesAvoidance>::SharedPtr
    set_obstacles_avoidance_service_;

  rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr request_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_SWITCH_OBSTACLES_AVOIDANCE_HPP
