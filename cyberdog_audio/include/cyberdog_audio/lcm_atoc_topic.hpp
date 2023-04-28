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
#ifndef CYBERDOG_AUDIO__LCM_ATOC_TOPIC_HPP_
#define CYBERDOG_AUDIO__LCM_ATOC_TOPIC_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_audio/instruction_def.hpp"

namespace cyberdog
{
namespace interaction
{
using ATOCTOPIC_HANDLER_CALLBACK =
  std::function<void (const lcm::ReceiveBuffer *, const std::string &,
    const audio_lcm::lcm_data *)>;
using AUDIO_CYBERDOG_TOPIC_FUNC = std::function<void (const std::string &)>;
class LcmAtocTopic
{
private:
  ATOCTOPIC_HANDLER_CALLBACK handler_callback;
  std::unique_ptr<lcm::LCM> lcm_;
  std::thread recv_message_thread;
  uint32_t recv_msg_cnt;
  bool exit;
  // std::map<const std::string, AUDIO_CYBERDOG_TOPIC_FUNC> audio_cyberdog_topic_cmd_map;

public:
  explicit LcmAtocTopic(ATOCTOPIC_HANDLER_CALLBACK callback_)
  : handler_callback(callback_), recv_msg_cnt(0),
    exit(false)
  {
    INFO("LcmAtocTopic");
    lcm_ = std::make_unique<lcm::LCM>(LCM_ADDR);
    recv_message_thread = std::thread(
      [this]() {
        if (!lcm_->good()) {
          ERROR("lcm recv topic is not good!");
          return;
        }
        lcm_->subscribe(
          ATOC_TOPIC, &LcmAtocTopic::LcmHandler,
          this);
        try {
          while (0 == lcm_->handle()) {
            // while (!exit && 0 == lcm_->handleTimeout(10)) {
            if (exit) {
              break;
            }
          }
        } catch (const std::exception & e) {
          std::cerr << e.what() << '\n';
        } catch (...) {
          ERROR("atoc topic expection!");
        }
      });
  }
  ~LcmAtocTopic()
  {
    exit = true;
    if (recv_message_thread.joinable()) {
      recv_message_thread.join();
    }
    lcm_.reset(nullptr);
    INFO("~LcmAtocTopic");
  }

private:
  void LcmHandler(
    const lcm::ReceiveBuffer * rbuf,
    const std::string & chan, const audio_lcm::lcm_data * msg)
  {
    try {
      if (msg == NULL) {
        ERROR("lcm handler:null pointer exception.");
        return;
      }
      INFO_MILLSECONDS(
        5000,
        "subcribe:%s(%d): cmd[%s], data[%s]",
        chan.c_str(), recv_msg_cnt,
        (msg->cmd.empty() ? "empty" : msg->cmd.c_str()),
        (msg->data.empty() ? "empty" : msg->data.c_str()));
      recv_msg_cnt++;
      handler_callback(rbuf, chan, msg);
    } catch (const std::exception & e) {
      std::cerr << ATOC_TOPIC << ":" << e.what() << '\n';
    } catch (...) {
      WARN("%s:unkown exception", ATOC_TOPIC);
    }
  }
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AUDIO__LCM_ATOC_TOPIC_HPP_
