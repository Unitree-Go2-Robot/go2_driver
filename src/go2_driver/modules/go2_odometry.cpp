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


#include <go2_driver/modules/go2_odometry.hpp>


namespace go2_driver
{

Go2Odometry::Go2Odometry(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node),
  tf_broadcaster_(node)
{
}

CallbackReturnT Go2Odometry::on_configure()
{
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/utlidar/robot_odom", 10,
    std::bind(&Go2Odometry::publish_odometry, this, std::placeholders::_1));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Odometry::on_activate()
{
  odom_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Odometry::on_deactivate()
{
  odom_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Odometry::on_cleanup()
{
  return CallbackReturnT::SUCCESS;
}

void Go2Odometry::publish_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = msg->header.stamp;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = msg->pose.pose.position.x;
  transform.transform.translation.y = msg->pose.pose.position.y;
  transform.transform.translation.z = msg->pose.pose.position.z;
  transform.transform.rotation.x = msg->pose.pose.orientation.x;
  transform.transform.rotation.y = msg->pose.pose.orientation.y;
  transform.transform.rotation.z = msg->pose.pose.orientation.z;
  transform.transform.rotation.w = msg->pose.pose.orientation.w;
  tf_broadcaster_.sendTransform(transform);

  if (!odom_published_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = msg->pose.pose.position.x;
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;
    odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_pub_->publish(odom);
    odom_published_ = true;
  }
}

}  // namespace go2_driver
