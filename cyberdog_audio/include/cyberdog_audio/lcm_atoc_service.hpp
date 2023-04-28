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
#ifndef CYBERDOG_AUDIO__LCM_ATOC_SERVICE_HPP_
#define CYBERDOG_AUDIO__LCM_ATOC_SERVICE_HPP_

#include <memory>
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_audio/instruction_def.hpp"

namespace cyberdog
{
namespace interaction
{
using ATOCSERVICE_HANDLER_CALLBACK =
  std::function<void (const audio_lcm::lcm_data &, audio_lcm::lcm_data &)>;
class LcmAtocService
{
  using LcmServer = cyberdog::common::LcmServer<audio_lcm::lcm_data, audio_lcm::lcm_data>;

private:
  ATOCSERVICE_HANDLER_CALLBACK handler_callback;
  std::unique_ptr<LcmServer> server;
  std::thread server_thread;
  bool exit;

public:
  explicit LcmAtocService(ATOCSERVICE_HANDLER_CALLBACK callback_)
  : handler_callback(callback_), exit(false)
  {
    INFO("LcmAtocService");
    server = std::make_unique<LcmServer>(
      ATOC_SERVICE,
      std::bind(
        &LcmAtocService::ServerCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
    server_thread = std::thread(
      [this]() {
        server->Spin();
      });
  }
  ~LcmAtocService()
  {
    server->Exit();
    if (server_thread.joinable()) {
      server_thread.join();
    }
    server.reset(nullptr);
    INFO("~LcmAtocService");
  }

private:
  void ServerCallback(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res)
  {
    try {
      INFO_MILLSECONDS(
        5000,
        "server receive: cmd[%s], data[%s]",
        (req.cmd.empty() ? "empty" : req.cmd.c_str()),
        (req.data.empty() ? "empty" : req.data.c_str()));
      if (req.cmd.empty()) {
        return;
      }
      handler_callback(req, res);
    } catch (const std::exception & e) {
      std::cerr << ATOC_SERVICE << ":" << e.what() << '\n';
    } catch (...) {
      WARN("%s:unkown exception", ATOC_SERVICE);
    }
  }
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AUDIO__LCM_ATOC_SERVICE_HPP_
