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


#ifndef GO2_DRIVER__GO2_ODOMETRY_HPP
#define GO2_DRIVER__GO2_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "go2_driver/utils/go2_lifecycle_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2Odometry : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2Odometry(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2Odometry() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void publish_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool odom_published_{false};
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_ODOMETRY_HPP
