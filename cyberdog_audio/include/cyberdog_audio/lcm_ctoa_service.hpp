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
#ifndef CYBERDOG_AUDIO__LCM_CTOA_SERVICE_HPP_
#define CYBERDOG_AUDIO__LCM_CTOA_SERVICE_HPP_

#include <string>
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_audio/instruction_def.hpp"

namespace cyberdog
{
namespace interaction
{
using CTOASERVICE_HANDLER_CALLBACK =
  std::function<bool (const audio_lcm::lcm_data &, audio_lcm::lcm_data &)>;
using LcmClient = cyberdog::common::LcmClient<audio_lcm::lcm_data, audio_lcm::lcm_data>;
using CYBERDOG_AUDIO_SERVICE_RETURN_FUNC = std::function<bool (const std::string &)>;

class LcmCtoaService
{
private:
  CTOASERVICE_HANDLER_CALLBACK handler_callback;
  // std::unique_ptr<LcmClient> client;
  uint32_t failed_times;
  // std::map<const std::string,
  //   CYBERDOG_AUDIO_SERVICE_RETURN_FUNC> cyberdog_audio_service_return_cmd_map;

public:
  explicit LcmCtoaService(CTOASERVICE_HANDLER_CALLBACK callback_)
  : handler_callback(callback_), failed_times(0)
  {
    INFO("LcmCtoaService");
    // client = std::make_unique<LcmClient>(CTOA_SERVICE);
  }
  ~LcmCtoaService()
  {
    // client.reset(nullptr);
    INFO("~LcmCtoaService");
  }
  bool ClientRequest(
    const audio_lcm::lcm_data & req,
    audio_lcm::lcm_data & res)
  {
    bool result = false;
    try {
      bool result = false;
      LcmClient client(CTOA_SERVICE);
      result = client.Request(req, res, 1000);
      if (!result) {
        ERROR("client thread failed %d times", ++failed_times);
      } else {
        // call handle function
        return handler_callback(req, res);
      }
    } catch (const std::exception & e) {
      std::cerr << CTOA_SERVICE << ":" << e.what() << '\n';
    } catch (...) {
      WARN("%s:unkown exception", CTOA_SERVICE);
    }
    return result;
  }
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AUDIO__LCM_CTOA_SERVICE_HPP_
