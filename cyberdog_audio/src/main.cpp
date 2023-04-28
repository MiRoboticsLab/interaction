// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <memory>
#include <string>
#include "cyberdog_audio/cyberdog_audio.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_debug/backtrace.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE(NODE_NAME);
  cyberdog::debug::register_signal();
  // std::string sn;
  // {
  //   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tmp_get_sn");
  //   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_srv_;
  //   audio_sn_ger_srv_ =
  //     node->create_client<std_srvs::srv::Trigger>("get_dog_sn");
  //   if (!audio_sn_ger_srv_->wait_for_service()) {
  //     ERROR("call sn server not avalible");
  //   }
  //   auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  //   auto future_result = audio_sn_ger_srv_->async_send_request(req);
  //   if (!(rclcpp::spin_until_future_complete(node, future_result) ==
  //     rclcpp::FutureReturnCode::SUCCESS))
  //   {
  //     ERROR("call get sn service failed!");
  //   }
  //   sn = future_result.get()->message;
  // }
  rclcpp::executors::MultiThreadedExecutor executor;
  auto audio_node =
    std::make_shared<cyberdog::interaction::CyberdogAudio>();
  executor.add_node(audio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
