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


#ifndef GO2_DRIVER__GO2_CAMERA_HPP
#define GO2_DRIVER__GO2_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "go2_driver/utils/go2_lifecycle_node.hpp"

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}


namespace go2_driver
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Go2Camera : public go2_driver::Go2LifecycleNode
{
public:
  explicit Go2Camera(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
  ~Go2Camera() override = default;

  CallbackReturnT on_configure() override;
  CallbackReturnT on_activate() override;
  CallbackReturnT on_deactivate() override;
  CallbackReturnT on_cleanup() override;

private:
  void front_video_data_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg);
  void publishCameraInfo();

  rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr front_video_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_publisher_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  AVCodec * codec_;
  AVCodecContext * p_codec_context_;
  AVPacket * sps_packet_;
  AVPacket * pps_packet_;
  bool sps_sent_{false};
  int camera_resolution_;

  std::string frame_id_;
  std::string distorsion_model_;
  int height_;
  int width_;
  std::vector<double> d_;
  std::vector<double> k_;
  std::vector<double> r_;
  std::vector<double> p_;
  int binning_x_;
  int binning_y_;
  int roi_x_offset_;
  int roi_y_offset_;
  int roi_height_;
  int roi_width_;
  bool roi_do_rectify_;

  // Precompute lens correction interpolation
  cv::Mat mapX_, mapY_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_CAMERA_HPP
