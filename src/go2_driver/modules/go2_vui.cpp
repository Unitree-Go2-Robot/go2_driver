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


#include <go2_driver/modules/go2_vui.hpp>


namespace go2_driver
{

Go2VUI::Go2VUI(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node)
{
}

CallbackReturnT Go2VUI::on_configure()
{
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>("api/vui/request", 10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mVUI module configured.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2VUI::on_activate()
{
  request_pub_->on_activate();

  get_brightness_service_ =
    node_->create_service<go2_interfaces::srv::GetBrightness>(
    "get_brightness",
    std::bind(
      &Go2VUI::handleGetBrightness, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_switch_service_ =
    node_->create_service<go2_interfaces::srv::GetSwitch>(
    "get_switch",
    std::bind(
      &Go2VUI::handleGetSwitch, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_volume_service_ =
    node_->create_service<go2_interfaces::srv::GetVolume>(
    "get_volume",
    std::bind(
      &Go2VUI::handleGetVolume, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_brightness_service_ =
    node_->create_service<go2_interfaces::srv::SetBrightness>(
    "set_brightness",
    std::bind(
      &Go2VUI::handleSetBrightness, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_switch_service_ =
    node_->create_service<go2_interfaces::srv::SetSwitch>(
    "set_switch",
    std::bind(
      &Go2VUI::handleSetSwitch, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_volume_service_ =
    node_->create_service<go2_interfaces::srv::SetVolume>(
    "set_volume",
    std::bind(
      &Go2VUI::handleSetVolume, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mVUI module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2VUI::on_deactivate()
{
  request_pub_->on_deactivate();

  get_brightness_service_.reset();
  get_switch_service_.reset();
  set_volume_service_.reset();
  set_brightness_service_.reset();
  set_switch_service_.reset();
  get_volume_service_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mVUI module deactivated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2VUI::on_cleanup()
{
  request_pub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mVUI module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2VUI::handleGetBrightness(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetBrightness::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetBrightness::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::GetBrightness);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/vui/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Vui::GetBrightness)) {
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
      response->message = "Timeout waiting for GetBrightness response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get brightness";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->brightness = data["brightness"];
  response->success = true;
  response->message = "Get brightness successfully";
}

void Go2VUI::handleGetSwitch(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetSwitch::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetSwitch::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::GetSwitch);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/vui/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Vui::GetSwitch)) {
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
      response->message = "Timeout waiting for GetSwitch response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get switch";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->enable = data["enable"];
  response->success = true;
  response->message = "Get switch successfully";
}

void Go2VUI::handleGetVolume(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetVolume::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetVolume::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::GetVolume);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/vui/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Vui::GetVolume)) {
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
      response->message = "Timeout waiting for GetVolume response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get volume";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->volume = data["volume"];
  response->success = true;
  response->message = "Get volume successfully";
}

void Go2VUI::handleSetBrightness(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SetBrightness::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SetBrightness::Response> response)
{
  (void)request_header;

  if (request->brightness < 0 || request->brightness > 10) {
    response->success = false;
    response->message = "Brightness value is out of range [0 ~ 10]";
    return;
  }

  nlohmann::json js;
  js["brightness"] = request->brightness;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::SetBrightness);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Set brightness successfully";
}

void Go2VUI::handleSetSwitch(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SetSwitch::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SetSwitch::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["enable"] = request->enable;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::SetSwitch);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Set switch successfully";
}

void Go2VUI::handleSetVolume(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SetVolume::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SetVolume::Response> response)
{
  (void)request_header;

  if (request->volume < 0 || request->volume > 10) {
    response->success = false;
    response->message = "Volume value is out of range [0 ~ 10]";
    return;
  }

  nlohmann::json js;
  js["volume"] = request->volume;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Vui::SetVolume);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Set volume successfully";
}

}  // namespace go2_driver
