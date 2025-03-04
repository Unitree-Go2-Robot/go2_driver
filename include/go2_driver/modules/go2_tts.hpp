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


#ifndef GO2_DRIVER__GO2_TTS_HPP
#define GO2_DRIVER__GO2_TTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include "go2_driver/utils/go2_api_id.hpp"
#include "unitree_api/msg/request.hpp"
#include "go2_interfaces/srv/say.hpp"
#include "nlohmann/json.hpp"
#include <vector>
#include <fstream>
#include <cstdlib>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>

namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2TTS : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2TTS(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2TTS() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void handleSay(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Say::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Say::Response> response);

  rclcpp::Service<go2_interfaces::srv::Say>::SharedPtr say_service_;

  rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr say_request_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  std::vector<std::vector<uint8_t>> calculate_chunks(
    const std::vector<uint8_t> & data,
    size_t chunk_size);
  std::vector<uint8_t> read_file(const std::string & filename);
  std::string base64_encode(const std::vector<uint8_t> & data);
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_TTS_HPP
