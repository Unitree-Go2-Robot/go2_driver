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


#ifndef GO2_DRIVER__GO2_HANDLE_SERVICES_HPP
#define GO2_DRIVER__GO2_HANDLE_SERVICES_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include "go2_interfaces/srv/body_height.hpp"
#include "go2_interfaces/srv/continuous_gait.hpp"
#include "go2_interfaces/srv/euler.hpp"
#include "go2_interfaces/srv/foot_raise_height.hpp"
#include "go2_interfaces/srv/mode.hpp"
#include "go2_interfaces/srv/pose.hpp"
#include "go2_interfaces/srv/speed_level.hpp"
#include "go2_interfaces/srv/switch_gait.hpp"
#include "go2_interfaces/srv/switch_joystick.hpp"
#include "go2_interfaces/srv/get_body_height.hpp"
#include "go2_interfaces/srv/get_foot_raise_height.hpp"
#include "go2_interfaces/srv/get_speed_level.hpp"
#include "go2_interfaces/srv/get_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "go2_driver/utils/go2_api_id.hpp"
#include "nlohmann/json.hpp"


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2HandleServices : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2HandleServices(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2HandleServices() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void handleBodyHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::BodyHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::BodyHeight::Response> response);

  void handleContinuousGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Request> request,
    const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Response> response);

  void handleEuler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Euler::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Euler::Response> response);

  void handleFootRaiseHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Response> response);

  void handleMode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Mode::Response> response);

  void handlePose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Pose::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Pose::Response> response);

  void handleSpeedLevel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response);

  void handleSwitchGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SwitchGait::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SwitchGait::Response> response);

  void handleSwitchJoystick(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response);

  void handleGetBodyHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetBodyHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetBodyHeight::Response> response);

  void handleGetFootRaiseHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetFootRaiseHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetFootRaiseHeight::Response> response);

  void handleGetSpeedLevel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetSpeedLevel::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetSpeedLevel::Response> response);

  void handleGetState(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::GetState::Request> request,
    const std::shared_ptr<go2_interfaces::srv::GetState::Response> response);

  rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr request_pub_;

  rclcpp::Service<go2_interfaces::srv::BodyHeight>::SharedPtr set_body_height_service_;
  rclcpp::Service<go2_interfaces::srv::ContinuousGait>::SharedPtr set_continuous_gait_service_;
  rclcpp::Service<go2_interfaces::srv::Euler>::SharedPtr set_euler_service_;
  rclcpp::Service<go2_interfaces::srv::FootRaiseHeight>::SharedPtr set_foot_raise_height_service_;
  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr set_mode_service_;
  rclcpp::Service<go2_interfaces::srv::Pose>::SharedPtr set_pose_service_;
  rclcpp::Service<go2_interfaces::srv::SpeedLevel>::SharedPtr set_speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchGait>::SharedPtr set_switch_gait_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchJoystick>::SharedPtr set_switch_joystick_service_;
  rclcpp::Service<go2_interfaces::srv::GetBodyHeight>::SharedPtr get_body_height_service_;
  rclcpp::Service<go2_interfaces::srv::GetFootRaiseHeight>::SharedPtr get_foot_raise_height_service_;
  rclcpp::Service<go2_interfaces::srv::GetSpeedLevel>::SharedPtr get_speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::GetState>::SharedPtr get_state_service_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_HANDLE_SERVICES_HPP
