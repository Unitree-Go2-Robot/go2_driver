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


#include <go2_driver/modules/go2_handle_services.hpp>


namespace go2_driver
{

Go2HandleServices::Go2HandleServices(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node)
{
}

CallbackReturnT Go2HandleServices::on_configure()
{
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>("api/sport/request", 10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mHandle services module configured.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2HandleServices::on_activate()
{
  request_pub_->on_activate();

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&Go2HandleServices::cmd_vel_callback, this, std::placeholders::_1));

  set_body_height_service_ =
    node_->create_service<go2_interfaces::srv::BodyHeight>(
    "body_height",
    std::bind(
      &Go2HandleServices::handleBodyHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_continuous_gait_service_ =
    node_->create_service<go2_interfaces::srv::ContinuousGait>(
    "continuous_gait",
    std::bind(
      &Go2HandleServices::handleContinuousGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_euler_service_ =
    node_->create_service<go2_interfaces::srv::Euler>(
    "euler",
    std::bind(
      &Go2HandleServices::handleEuler, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_foot_raise_height_service_ =
    node_->create_service<go2_interfaces::srv::FootRaiseHeight>(
    "foot_raise_height",
    std::bind(
      &Go2HandleServices::handleFootRaiseHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_mode_service_ =
    node_->create_service<go2_interfaces::srv::Mode>(
    "mode",
    std::bind(
      &Go2HandleServices::handleMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_pose_service_ =
    node_->create_service<go2_interfaces::srv::Pose>(
    "pose",
    std::bind(
      &Go2HandleServices::handlePose, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_speed_level_service_ =
    node_->create_service<go2_interfaces::srv::SpeedLevel>(
    "speed_level",
    std::bind(
      &Go2HandleServices::handleSpeedLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_switch_gait_service_ =
    node_->create_service<go2_interfaces::srv::SwitchGait>(
    "switch_gait",
    std::bind(
      &Go2HandleServices::handleSwitchGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_switch_joystick_service_ =
    node_->create_service<go2_interfaces::srv::SwitchJoystick>(
    "switch_joystick",
    std::bind(
      &Go2HandleServices::handleSwitchJoystick, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_body_height_service_ =
    node_->create_service<go2_interfaces::srv::GetBodyHeight>(
    "get_body_height",
    std::bind(
      &Go2HandleServices::handleGetBodyHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_foot_raise_height_service_ =
    node_->create_service<go2_interfaces::srv::GetFootRaiseHeight>(
    "get_foot_raise_height",
    std::bind(
      &Go2HandleServices::handleGetFootRaiseHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_speed_level_service_ =
    node_->create_service<go2_interfaces::srv::GetSpeedLevel>(
    "get_speed_level",
    std::bind(
      &Go2HandleServices::handleGetSpeedLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_state_service_ =
    node_->create_service<go2_interfaces::srv::GetState>(
    "get_state",
    std::bind(
      &Go2HandleServices::handleGetState, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mHandle services module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2HandleServices::on_deactivate()
{
  request_pub_->on_deactivate();

  cmd_vel_sub_.reset();
  set_body_height_service_.reset();
  set_continuous_gait_service_.reset();
  set_euler_service_.reset();
  set_foot_raise_height_service_.reset();
  set_mode_service_.reset();
  set_pose_service_.reset();
  set_speed_level_service_.reset();
  set_switch_gait_service_.reset();
  set_switch_joystick_service_.reset();
  get_body_height_service_.reset();
  get_foot_raise_height_service_.reset();
  get_speed_level_service_.reset();
  get_state_service_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mHandle services module deactivated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2HandleServices::on_cleanup()
{
  request_pub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mHandle services module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2HandleServices::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nlohmann::json js;
  js["x"] = msg->linear.x;
  js["y"] = msg->linear.y;
  js["z"] = msg->angular.z;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Move);

  request_pub_->publish(req);
}

void Go2HandleServices::handleBodyHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::BodyHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::BodyHeight::Response> response)
{
  (void)request_header;

  if (request->height < -0.18 || request->height > 0.03) {
    response->success = false;
    response->message = "Height value is out of range [-0.18 ~ 0.03]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->height;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::BodyHeight);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Body height changed";
}

void Go2HandleServices::handleContinuousGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Request> request,
  const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::ContinuousGait);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleEuler(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Euler::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Euler::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  if (request->roll < -0.75 || request->roll > 0.75) {
    response->success = false;
    response->message = "Roll value is out of range [-0.75 ~ 0.75]";
    return;
  } else if (request->pitch < -0.75 || request->pitch > 0.75) {
    response->success = false;
    response->message = "Pitch value is out of range [-0.75 ~ 0.75]";
    return;
  } else if (request->yaw < -0.6 || request->yaw > 0.6) {
    response->success = false;
    response->message = "Yaw value is out of range [-0.6 ~ 0.6]";
    return;
  }

  js["x"] = request->roll;
  js["y"] = request->pitch;
  js["z"] = request->yaw;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Euler);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleFootRaiseHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Response> response)
{
  (void)request_header;

  if (request->height < -0.06 || request->height > 0.03) {
    response->success = false;
    response->message = "Height value is out of range [-0.06 ~ 0.03]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->height;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FootRaiseHeight);

  request_pub_->publish(req);
  response->success = true;
  response->message = "Foot raise height changed";
}

void Go2HandleServices::handleMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Mode::Response> response)
{
  (void)request_header;
  std::string mode = request->mode;

  unitree_api::msg::Request req;

  if (mode == "damp") {
    response->message = "Change the mode to Damp";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Damp);
  } else if (mode == "balance_stand") {
    response->message = "Change the mode to BalanceStand";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::BalanceStand);
  } else if (mode == "stop_move") {
    response->message = "Change the mode to StopMove";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StopMove);
  } else if (mode == "stand_up") {
    response->message = "Change the mode to StandUp";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StandUp);
  } else if (mode == "stand_down") {
    response->message = "Change the mode to StandDown";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StandDown);
  } else if (mode == "sit") {
    response->message = "Change the mode to Sit";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Sit);
  } else if (mode == "rise_sit") {
    response->message = "Change the mode to RiseSit";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::RiseSit);
  } else if (mode == "hello") {
    response->message = "Change the mode to Hello. Say hello to your robot!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Hello);
  } else if (mode == "stretch") {
    response->message = "Change the mode to Stretch";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Stretch);
  } else if (mode == "wallow") {
    response->message = "Change the mode to Wallow";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Wallow);
  } else if (mode == "scrape") {
    response->message = "Change the mode to Scrape";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Scrape);
  } else if (mode == "front_flip") {
    response->message = "Front flip??? Really? You want to break your robot? Crazy!";
    // req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontFlip);
  } else if (mode == "front_jump") {
    response->message = "Change the mode to Front Jump";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontJump);
  } else if (mode == "front_pounce") {
    response->message = "Change the mode to Front Pounce";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontPounce);
  } else if (mode == "dance1") {
    response->message = "Change the mode to Dance 1. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance1);
  } else if (mode == "dance2") {
    response->message = "Change the mode to Dance 2. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance2);
  } else if (mode == "finger_heart") {
    response->message = "Change the mode to Finger Heart";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FingerHeart);
  } else if (mode == "finger_heart") {
    response->message = "Change the mode to Finger Heart";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FingerHeart);
  } else if (mode == "dance3") {
    response->message = "Change the mode to Dance 3. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance3);
  } else if (mode == "dance4") {
    response->message = "Change the mode to Dance 4. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance4);
  } else if (mode == "hop_spin_left") {
    response->message = "Change the mode to Hop Spin Left";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::HopSpinLeft);
  } else if (mode == "hop_spin_right") {
    response->message = "Change the mode to Hop Spin Right";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::HopSpinRight);
  } else if (mode == "left_flip") {
    response->message = "Left flip??? Really? You want to break your robot? Crazy!";
    // req.header.identity.api_id = static_cast<int>(go2_driver::Mode::LeftFlip);
  } else if (mode == "back_flip") {
    response->message = "Back flip??? Really? You want to break your robot? Crazy!";
    // req.header.identity.api_id = static_cast<int>(go2_driver::Mode::BackFlip);
  } else if (mode == "free_walk") {
    response->message = "Change the mode to Free Walk";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FreeWalk);
  } else if (mode == "free_bound") {
    response->message = "Change the mode to Free Bound";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FreeBound);
  } else if (mode == "free_jump") {
    response->message = "Change the mode to Free Jump";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FreeJump);
  } else if (mode == "free_avoid") {
    response->message = "Change the mode to Free Avoid";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FreeAvoid);
  } else if (mode == "walk_stair") {
    response->message = "Change the mode to Walk Stair";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::WalkStair);
  } else if (mode == "walk_up_right") {
    response->message = "Change the mode to Walk Up Right";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::WalkUpRight);
  } else if (mode == "cross_step") {
    response->message = "Change the mode to Cross Step";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::CrossStep);
  } else {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handlePose(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Pose::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Pose::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Pose);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleSpeedLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response)
{
  (void)request_header;

  if (request->level < -1 || request->level > 1) {
    response->success = false;
    response->message = "Speed level is out of range [-1 ~ 1]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->level;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SpeedLevel);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleSwitchGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SwitchGait::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SwitchGait::Response> response)
{
  (void)request_header;

  if (request->d < 0 || request->d > 4) {
    response->success = false;
    response->message = "Invalid gait type [0 - 4]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->d;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SwitchGait);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleSwitchJoystick(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SwitchJoystick);

  request_pub_->publish(req);
  response->success = true;
}

void Go2HandleServices::handleGetBodyHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetBodyHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetBodyHeight::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::GetBodyHeight);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/sport/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Mode::GetBodyHeight)) {
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
      response->message = "Timeout waiting for GetBodyHeight response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get body height";
    return;
  }

  auto data = js.parse(response_msg->data);
  // response->height = data["data"];
  response->success = true;
  response->message = "Get body height successfully";
}


void Go2HandleServices::handleGetFootRaiseHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetFootRaiseHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetFootRaiseHeight::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::GetFootRaiseHeight);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/sport/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Mode::GetFootRaiseHeight)) {
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
      response->message = "Timeout waiting for GetFootRaiseHeight response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get foot raise height";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->height = data["data"];
  response->success = true;
  response->message = "Get foot raise height successfully";
}

void Go2HandleServices::handleGetSpeedLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetSpeedLevel::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetSpeedLevel::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::GetSpeedLevel);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = aux_node->create_subscription<unitree_api::msg::Response>(
    "/api/sport/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Mode::GetSpeedLevel)) {
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
      response->message = "Timeout waiting for GetSpeedLevel response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get speed level";
    return;
  }

  auto data = js.parse(response_msg->data);
  response->level = data["data"];
  response->success = true;
  response->message = "Get speed level successfully";
}

void Go2HandleServices::handleGetState(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::GetState::Request> request,
  const std::shared_ptr<go2_interfaces::srv::GetState::Response> response)
{
  (void)request_header;
  (void)request;

  nlohmann::json js;
  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::GetState);

  unitree_api::msg::Response::SharedPtr response_msg = nullptr;
  rclcpp::Node::SharedPtr aux_node = rclcpp::Node::make_shared("aux_node");

  auto response_sub_ = node_->create_subscription<unitree_api::msg::Response>(
    "/api/sport/response", 10,
    [this, &response_msg](const unitree_api::msg::Response::SharedPtr msg) {
      if (msg->header.identity.api_id == static_cast<int>(go2_driver::Mode::GetState)) {
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
      response->message = "Timeout waiting for GetState response";
      return;
    }
  }

  if (response_msg->header.status.code != 0) {
    response->success = false;
    response->message = "Failed to get state";
    return;
  }

  auto data = js.parse(response_msg->data);
  // response->state = data["data"];  // TODO(Juancams): Look at what field it is
  response->success = true;
  response->message = "Get state successfully";
}

}  // namespace go2_driver
