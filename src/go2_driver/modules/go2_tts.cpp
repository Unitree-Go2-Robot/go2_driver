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

#include <go2_driver/modules/go2_tts.hpp>


namespace go2_driver
{

Go2TTS::Go2TTS(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
: node_(node)
{
}

CallbackReturnT Go2TTS::on_configure()
{
  say_request_pub_ = node_->create_publisher<unitree_api::msg::Request>("api/audiohub/request", 10);

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mTTS module configured.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2TTS::on_activate()
{
  say_request_pub_->on_activate();

  say_service_ =
    node_->create_service<go2_interfaces::srv::Say>(
    "say",
    std::bind(
      &Go2TTS::handleSay, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mTTS module activated.\033[0m");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2TTS::on_deactivate()
{
  say_request_pub_->on_deactivate();

  say_service_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Go2TTS::on_cleanup()
{
  say_request_pub_.reset();

  RCLCPP_INFO(node_->get_logger(), "\033[1;34mTTS module cleaned up.\033[0m");

  return CallbackReturnT::SUCCESS;
}

void Go2TTS::handleSay(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Say::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Say::Response> response)
{
  (void)request_header;

  std::string text = request->text;
  std::string command = "echo \"" + text + "\" | text2wave -o /tmp/output.wav";

  system(command.c_str());

  std::vector<uint8_t> audio_data = read_file("/tmp/output.wav");
  if (audio_data.empty()) {
    response->success = false;
    return;
  }

  std::vector<std::vector<uint8_t>> chunks = calculate_chunks(audio_data, 256 * 1024);

  size_t total_chunks = chunks.size();

  unitree_api::msg::Request start_req;
  start_req.header.identity.api_id = static_cast<int>(go2_driver::Audio::StartAudio);

  // We waited a while for the open to be published
  say_request_pub_->publish(start_req);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  for (size_t chunk_idx = 0; chunk_idx < total_chunks; ++chunk_idx) {
    nlohmann::json j;
    j["current_block_index"] = chunk_idx + 1;
    j["total_block_number"] = total_chunks;
    j["block_content"] = base64_encode(chunks[chunk_idx]);

    unitree_api::msg::Request req;
    req.header.identity.api_id = static_cast<int>(go2_driver::Audio::TTS);

    req.parameter = j.dump();

    say_request_pub_->publish(req);
  }

  response->success = true;
}

std::vector<uint8_t> Go2TTS::read_file(const std::string & filename)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file) {
    std::cerr << "Error open: " << filename << std::endl;
    return {};
  }

  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

std::vector<std::vector<uint8_t>> Go2TTS::calculate_chunks(
  const std::vector<uint8_t> & data,
  size_t chunk_size)
{
  std::vector<std::vector<uint8_t>> chunks;

  for (size_t i = 0; i < data.size(); i += chunk_size) {
    size_t end = std::min(i + chunk_size, data.size());
    std::vector<uint8_t> chunk(data.begin() + i, data.begin() + end);
    chunks.push_back(chunk);
  }

  return chunks;
}

std::string Go2TTS::base64_encode(const std::vector<uint8_t> & data)
{
  BIO * bio, * b64;
  BUF_MEM * bufferPtr;

  b64 = BIO_new(BIO_f_base64());
  bio = BIO_new(BIO_s_mem());
  bio = BIO_push(b64, bio);
  BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);

  BIO_write(bio, data.data(), data.size());
  BIO_flush(bio);
  BIO_get_mem_ptr(bio, &bufferPtr);

  std::string encoded(bufferPtr->data, bufferPtr->length);
  BIO_free_all(bio);

  return encoded;
}

}  // namespace go2_driver
