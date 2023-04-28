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
#ifndef CYBERDOG_AUDIO__LCM_CTOA_TOPIC_HPP_
#define CYBERDOG_AUDIO__LCM_CTOA_TOPIC_HPP_

#include <thread>
#include <memory>
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_audio/instruction_def.hpp"

namespace cyberdog
{
namespace interaction
{
using CTOATOPIC_HANDLER_CALLBACK =
  std::function<int32_t(const std::shared_ptr<audio_lcm::lcm_data> &)>;
class LcmCtoaTopic
{
private:
  CTOATOPIC_HANDLER_CALLBACK handler_callback;
  std::unique_ptr<lcm::LCM> lcm_;
  std::thread send_message_thread;
  uint32_t send_msg_cnt;
  bool exit;

public:
  explicit LcmCtoaTopic(CTOATOPIC_HANDLER_CALLBACK callback_)
  : handler_callback(callback_), send_msg_cnt(0),
    exit(false)
  {
    INFO("LcmCtoaTopic");
    lcm_ = std::make_unique<lcm::LCM>(LCM_ADDR);
    send_message_thread = std::thread(
      [this]() {
        if (!lcm_->good()) {
          ERROR("lcm send topic is not good!");
          return;
        }
        try {
          // while (!exit && 0 == lcm_->handleTimeout(10)) {
          while (0 == lcm_->handle()) {
            if (exit) {
              break;
            }
          }
        } catch (const std::exception & e) {
          std::cerr << e.what() << '\n';
        } catch (...) {
          ERROR("ctoa topic expection!");
        }
      });
  }
  ~LcmCtoaTopic()
  {
    exit = true;
    if (send_message_thread.joinable()) {
      send_message_thread.join();
    }
    lcm_.reset(nullptr);
    INFO("~LcmCtoaTopic");
  }
  void LcmPublish(const std::shared_ptr<audio_lcm::lcm_data> & data)
  {
    if (!lcm_->good()) {
      INFO("publish encounter lcm is not good");
      return;
    }
    try {
      int32_t result = handler_callback(data);
      if (result != 0) {
        return;
      }
      INFO_MILLSECONDS(
        5000,
        "publish(%d): cmd[%s], data[%s]", send_msg_cnt,
        data->cmd.c_str(), data->data.c_str());
      lcm_->publish(CTOA_TOPIC, data.get());
      send_msg_cnt++;
    } catch (const std::exception & e) {
      std::cerr << CTOA_TOPIC << ":" << e.what() << '\n';
    } catch (...) {
      WARN("%s:unkown exception", CTOA_TOPIC);
    }
  }
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AUDIO__LCM_CTOA_TOPIC_HPP_
