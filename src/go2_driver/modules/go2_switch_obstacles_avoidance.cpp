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


#include <go2_driver/modules/go2_switch_obstacles_avoidance.hpp>


namespace go2_driver
{

Go2SwitchObstaclesAvoidance::Go2SwitchObstaclesAvoidance(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node)
{
}

CallbackReturnT Go2SwitchObstaclesAvoidance::on_configure()
{
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>(
    "api/obstacles_avoid/request",
    10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mObstacles Avoidance module configured.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2SwitchObstaclesAvoidance::on_activate()
{
  request_pub_->on_activate();

  get_obstacles_avoidance_service_ =
    node_->create_service<go2_interfaces::srv::GetSwitchObstaclesAvoidance>(
    "get_obstacles_avoidance",
    std::bind(
      &Go2SwitchObstaclesAvoidance::handleGetObstaclesAvoidance, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_obstacles_avoidance_service_ =
    node_->create_service<go2_interfaces::srv::SetSwitchObstaclesAvoidance>(
    "set_obstacles_avoidance",
    std::bind(
      &Go2SwitchObstaclesAvoidance::handleSetObstaclesAvoidance, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mObstacles Avoidance module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2SwitchObstaclesAvoidance::on_deactivate()
{
  request_pub_->on_deactivate();

  get_obstacles_avoidance_service_.reset();
  set_obstacles_avoidance_service_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mObstacles Avoidance module deactivated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2SwitchObstaclesAvoidance::on_cleanup()
{
  request_pub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mObstacles Avoidance module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2SwitchObstaclesAvoidance::handleGetObstaclesAvoidance(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetSwitchObstaclesAvoidance::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetSwitchObstaclesAvoidance::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::ObstaclesAvoidance::GetSwitch);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/obstacles_avoid/response", 10,
    [this, &response_msg](
      const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::ObstaclesAvoidance::GetSwitch)) {
        response_msg = msg;
      }
    });

  request_pub_->publish(req);

  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(5);

  while (response_msg == nullptr) {
    rclcpp::spin_some(aux_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto now = std::chrono::steady_clock::now();
    if (now - start_time > timeout) {
      response->success = false;
      response->message = "Timeout waiting for GetObstaclesAvoidance response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get switch obstacles avoidance";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->enable = data["enable"];
  response->success = true;
  response->message = "Get switch obstacles avoidance successfully";
}

void Go2SwitchObstaclesAvoidance::handleSetObstaclesAvoidance(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SetSwitchObstaclesAvoidance::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SetSwitchObstaclesAvoidance::Response> response)
{
  (void)request_header;

  if (request->enable < 0 || request->enable > 1) {
    response->success = false;
    response->message = "Switch value is out of range [0 ~ 1]";
    return;
  }

  nlohmann::json js;
  js["enable"] = request->enable;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::ObstaclesAvoidance::SetSwitch);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Set switch obstacles avoidance successfully";
}

}  // namespace go2_driver
