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
  node_->get_parameter("camera_resolution", camera_resolution_);
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

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_activate()
{
  image_publisher_->on_activate();

  front_video_sub_ = node_->create_subscription<unitree_go::msg::Go2FrontVideoData>(
    "frontvideostream", 10,
    std::bind(&Go2Camera::front_video_data_callback, this, std::placeholders::_1));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_deactivate()
{
  image_publisher_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2Camera::on_cleanup()
{
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

  auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
  image_msg->header.stamp = node_->get_clock()->now();
  image_msg->header.frame_id = "camera_frame";

  image_publisher_->publish(*image_msg);

  av_frame_free(&frame);
}

}  // namespace go2_driver
