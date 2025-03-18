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


#ifndef GO2_DRIVER__GO2_VUI_HPP
#define GO2_DRIVER__GO2_VUI_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include "go2_driver/utils/go2_api_id.hpp"

#include "go2_interfaces/srv/get_brightness.hpp"
#include "go2_interfaces/srv/get_switch.hpp"
#include "go2_interfaces/srv/get_volume.hpp"
#include "go2_interfaces/srv/set_brightness.hpp"
#include "go2_interfaces/srv/set_switch.hpp"
#include "go2_interfaces/srv/set_volume.hpp"

#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

#include "nlohmann/json.hpp"


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2VUI : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2VUI(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2VUI() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void handleGetBrightness(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetBrightness::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetBrightness::Response> response);

  void handleGetSwitch(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetSwitch::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetSwitch::Response> response);

  void handleGetVolume(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetVolume::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetVolume::Response> response);

  void handleSetBrightness(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SetBrightness::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SetBrightness::Response> response);

  void handleSetSwitch(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SetSwitch::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SetSwitch::Response> response);

  void handleSetVolume(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SetVolume::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SetVolume::Response> response);

  rclcpp::Service<go2_interfaces::srv::GetBrightness>::SharedPtr get_brightness_service_;
  rclcpp::Service<go2_interfaces::srv::GetSwitch>::SharedPtr get_switch_service_;
  rclcpp::Service<go2_interfaces::srv::GetVolume>::SharedPtr get_volume_service_;
  rclcpp::Service<go2_interfaces::srv::SetBrightness>::SharedPtr set_brightness_service_;
  rclcpp::Service<go2_interfaces::srv::SetSwitch>::SharedPtr set_switch_service_;
  rclcpp::Service<go2_interfaces::srv::SetVolume>::SharedPtr set_volume_service_;

  rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr request_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_VUI_HPP
