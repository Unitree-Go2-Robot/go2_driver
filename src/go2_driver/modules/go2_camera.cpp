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


#include <go2_driver/modules/go2_camera.hpp>

namespace go2_driver
{

Go2Camera::Go2Camera(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node),
  codec_(),
  p_codec_context_(),
  sps_packet_(),
  pps_packet_()
{
  node_->declare_parameter("camera_resolution", 720);
  node_->declare_parameter("frame_id", "camera_frame");
  node_->declare_parameter("height", 720);
  node_->declare_parameter("width", 1280);
  node_->declare_parameter("distorsion_model", "plumb_bob");
  node_->declare_parameter("d", std::vector<double>());
  node_->declare_parameter("k", std::vector<double>());
  node_->declare_parameter("r", std::vector<double>());
  node_->declare_parameter("p", std::vector<double>());
  node_->declare_parameter("binning_x", 0);
  node_->declare_parameter("binning_y", 0);
  node_->declare_parameter("roi.x_offset", 0);
  node_->declare_parameter("roi.y_offset", 0);
  node_->declare_parameter("roi.height", 0);
  node_->declare_parameter("roi.width", 0);
  node_->declare_parameter("roi.do_rectify", false);

  node_->get_parameter("camera_resolution", camera_resolution_);
  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("height", height_);
  node_->get_parameter("width", width_);
  node_->get_parameter("distorsion_model", distorsion_model_);
  node_->get_parameter("d", d_);
  node_->get_parameter("k", k_);
  node_->get_parameter("r", r_);
  node_->get_parameter("p", p_);
  node_->get_parameter("binning_x", binning_x_);
  node_->get_parameter("binning_y", binning_y_);
  node_->get_parameter("roi.x_offset", roi_x_offset_);
  node_->get_parameter("roi.y_offset", roi_y_offset_);
  node_->get_parameter("roi.height", roi_height_);
  node_->get_parameter("roi.width", roi_width_);
  node_->get_parameter("roi.do_rectify", roi_do_rectify_);
}

CallbackReturnT Go2Camera::on_configure()
{
  codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to find codec.");
    return CallbackReturnT::FAILURE;
  }

  p_codec_context_ = avcodec_alloc_context3(codec_);
  if (p_codec_context_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate codec context.");
    return CallbackReturnT::FAILURE;
  }

  if (avcodec_open2(p_codec_context_, codec_, nullptr) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open codec.");
    return CallbackReturnT::FAILURE;
  }

  image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
  camera_info_publisher_ =
    node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mCamera module configured.\033[0m");

  // Precompute undistort map for lens correction 
  cv::initUndistortRectifyMap(k_, d_, cv::Matx33f::eye(), k_, cv::SIZE(width_, height_), CV_32FC1, mapX_, mapY_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_activate()
{
  image_publisher_->on_activate();
  camera_info_publisher_->on_activate();

  front_video_sub_ = node_->create_subscription<unitree_go::msg::Go2FrontVideoData>(
    "frontvideostream", 10,
    std::bind(&Go2Camera::front_video_data_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mCamera module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_deactivate()
{
  image_publisher_->on_deactivate();
  camera_info_publisher_->on_deactivate();

  front_video_sub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mCamera module deactivated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_cleanup()
{
  image_publisher_.reset();
  camera_info_publisher_.reset();

  avcodec_free_context(&p_codec_context_);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mCamera module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2Camera::front_video_data_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg)
{
  if (msg->resolution != camera_resolution_) {return;}
  AVPacket * packet = av_packet_alloc();

  av_new_packet(packet, msg->data.size());
  memcpy(packet->data, msg->data.data(), msg->data.size());

  if (packet->size >= 4 && packet->data[0] == 0x00 && packet->data[1] == 0x00 &&
    packet->data[2] == 0x00 && packet->data[3] == 0x01)
  {

    uint8_t nal_unit_type = packet->data[4] & 0x1F;

    if (nal_unit_type == 7) {
      if (!sps_packet_) {
        sps_packet_ = av_packet_alloc();
        av_packet_ref(sps_packet_, packet);
        RCLCPP_DEBUG(node_->get_logger(), "SPS received.");
      }
    } else if (nal_unit_type == 1) {
      if (!pps_packet_) {
        pps_packet_ = av_packet_alloc();
        av_packet_ref(pps_packet_, packet);
        RCLCPP_DEBUG(node_->get_logger(), "PPS received.");
      }
    }
  }

  if (!sps_packet_ || !pps_packet_) {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for SPS/PPS before decoding.");
    return;
  }

  if (!sps_sent_) {
    avcodec_send_packet(p_codec_context_, sps_packet_);
    avcodec_send_packet(p_codec_context_, pps_packet_);
    sps_sent_ = true;
  }

  int ret = avcodec_send_packet(p_codec_context_, packet);

  if (ret < 0) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errbuf, sizeof(errbuf));
    RCLCPP_DEBUG(node_->get_logger(), "Failed to send packet to decoder: %s", errbuf);
  }

  AVFrame * frame = av_frame_alloc();
  ret = avcodec_receive_frame(p_codec_context_, frame);

  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
    av_frame_free(&frame);
    return;
  } else if (ret < 0) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errbuf, sizeof(errbuf));
    RCLCPP_DEBUG(node_->get_logger(), "Failed to receive frame from decoder: %s", errbuf);
    av_frame_free(&frame);
    return;
  }

  cv::Mat yuv420p(frame->height * 3 / 2, frame->width, CV_8UC1);
  memcpy(yuv420p.data, frame->data[0], frame->linesize[0] * frame->height);
  memcpy(
    yuv420p.data + frame->linesize[0] * frame->height, frame->data[1],
    frame->linesize[1] * frame->height / 2);
  memcpy(
    yuv420p.data + frame->linesize[0] * frame->height + frame->linesize[1] * frame->height / 2,
    frame->data[2], frame->linesize[2] * frame->height / 2);

  cv::Mat bgr;
  cv::cvtColor(yuv420p, bgr, cv::COLOR_YUV420p2RGB);

  cv::Mat dst;
  // Undistort image using remap to improve performance
  cv::remap(bgr, dst, mapX_, mapY_, cv::INTER_LINEAR);

  auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dst).toImageMsg();
  image_msg->header.stamp = node_->get_clock()->now();
  image_msg->header.frame_id = "camera_frame";

  image_publisher_->publish(*image_msg);

  if (camera_info_publisher_->get_subscription_count() > 0) {
    publishCameraInfo();
  }

  av_frame_free(&frame);
}

void Go2Camera::publishCameraInfo()
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.stamp = node_->get_clock()->now();
  camera_info_msg.header.frame_id = frame_id_;
  camera_info_msg.height = height_;
  camera_info_msg.width = width_;
  camera_info_msg.distortion_model = distorsion_model_;
  camera_info_msg.d = std::vector<double>(d_.begin(), d_.end());
  std::copy(k_.begin(), k_.end(), camera_info_msg.k.begin());
  std::copy(r_.begin(), r_.end(), camera_info_msg.r.begin());
  std::copy(p_.begin(), p_.end(), camera_info_msg.p.begin());
  camera_info_msg.binning_x = binning_x_;
  camera_info_msg.binning_y = binning_y_;
  camera_info_msg.roi.x_offset = roi_x_offset_;
  camera_info_msg.roi.y_offset = roi_y_offset_;
  camera_info_msg.roi.height = roi_height_;
  camera_info_msg.roi.width = roi_width_;
  camera_info_msg.roi.do_rectify = roi_do_rectify_;

  camera_info_publisher_->publish(camera_info_msg);
}

}  // namespace go2_driver
