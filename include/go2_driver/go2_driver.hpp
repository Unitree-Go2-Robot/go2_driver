// Copyright 2024 Intelligent Robotics Lab
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

#ifndef GO2_DRIVER__GO2_DRIVER_HPP_
#define GO2_DRIVER__GO2_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"
#include "go2_driver/go2_api_id.hpp"
#include "go2_interfaces/srv/body_height.hpp"
#include "go2_interfaces/srv/continuous_gait.hpp"
#include "go2_interfaces/srv/euler.hpp"
#include "go2_interfaces/srv/foot_raise_height.hpp"
#include "go2_interfaces/srv/mode.hpp"
#include "go2_interfaces/srv/pose.hpp"
#include "go2_interfaces/srv/speed_level.hpp"
#include "go2_interfaces/srv/switch_gait.hpp"
#include "go2_interfaces/srv/switch_joystick.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}

namespace go2_driver
{

class Go2Driver : public rclcpp::Node
{
public:
  Go2Driver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void front_video_data_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg);

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

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr front_video_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<unitree_go::msg::IMUState>::SharedPtr imu_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  rclcpp::Service<go2_interfaces::srv::BodyHeight>::SharedPtr set_body_height_service_;
  rclcpp::Service<go2_interfaces::srv::ContinuousGait>::SharedPtr set_continuous_gait_service_;
  rclcpp::Service<go2_interfaces::srv::Euler>::SharedPtr set_euler_service_;
  rclcpp::Service<go2_interfaces::srv::FootRaiseHeight>::SharedPtr set_foot_raise_height_service_;
  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr set_mode_service_;
  rclcpp::Service<go2_interfaces::srv::Pose>::SharedPtr set_pose_service_;
  rclcpp::Service<go2_interfaces::srv::SpeedLevel>::SharedPtr set_speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchGait>::SharedPtr set_switch_gait_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchJoystick>::SharedPtr set_switch_joystick_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_lidar_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  int camera_resolution_;
  sensor_msgs::msg::Joy joy_state_;
  AVCodec * codec_;
  AVCodecContext * p_codec_context_;
  AVPacket * sps_packet_;
  AVPacket * pps_packet_;

  bool sps_sent_{false};
  bool odom_published_{false};
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_DRIVER_HPP_
