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
#ifndef CYBERDOG_AUDIO__VOICE_CONTROL_HPP_
#define CYBERDOG_AUDIO__VOICE_CONTROL_HPP_

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <thread>
#include "cyberdog_audio/cyberdog_action.hpp"
#include "xpack/json.h"

namespace cyberdog
{
namespace interaction
{
struct local_data
{
  std::string action;
  uint8_t count;
  XPACK(O(action, count))
};
struct online_data
{
  std::string action;
  uint8_t count;
  std::string dialogid;
  std::string emotion;
  XPACK(O(action, count, dialogid, emotion))
};
struct wakeup_data
{
  float direction;
  XPACK(O(direction))
};
struct environment_data
{
  std::vector<std::string> events;
  std::vector<float> score;
  XPACK(O(events, score))
};
class VoiceControl
{
public:
  explicit VoiceControl(bool ae)
  : ptr_action(std::make_shared<CyberdogAction>()),
    action_enable(ae)
  {
    t_spin = std::thread(
      [this]() {
        rclcpp::spin(ptr_action);
      });
    t_spin.detach();
  }
  ~VoiceControl() = default;

public:
  void Local(const std::string & data)
  {
    INFO("local:%s", data.c_str());
    local_data ld;
    xpack::json::decode(data, ld);
    if (action_enable) {
      ptr_action->ExecuteAction(ld.action, ld.count);
    }
  }

  void Online(const std::string & data)
  {
    INFO("online:%s", data.c_str());
    online_data od;
    xpack::json::decode(data, od);
    if (action_enable) {
      ptr_action->ExecuteSpecialAction(
        od.action, od.count,
        od.dialogid, od.emotion);
    }
  }
  void Personal(std::string type, std::string value)
  {
    std::string motion_id;
    if (std::strcmp(type.c_str(), "motion") == 0) {
      int value_ = std::atoi(value.c_str());
      switch (value_) {
        case 1:
          motion_id = "AND_LAZY";
          break;
        case 2:
          motion_id = "LEFT_HAND";
          break;
        case 3:
          motion_id = "RIGHT_HAND";
          break;
        default:
          INFO("personal  value(TrainPlan.msg) is NULL");
          break;
      }
    }
    INFO("motion_id = %s", motion_id.c_str());
    if (!motion_id.empty()) {
      INFO("personal voice control is %s", motion_id.c_str());
      ptr_action->ExecuteAction(motion_id, 1);
    } else {
      INFO("personal voice control is null");
    }
  }
  void Wakeup(const std::string & data)
  {
    INFO("wakeup:%s", data.c_str());
    wakeup_data wd;
    xpack::json::decode(data, wd);
    ptr_action->WakeUp(wd.direction);
  }

  void Environment(const std::string & data)
  {
    environment_data ed;
    xpack::json::decode(data, ed);
    // 对环境音作出反映
  }

  void Instruction(const std::string & data)
  {
    // data是个json字符串，语音板负责转发小爱服务器的数据
    (void) data;
  }

  void SetAcitionControl(bool enable)
  {
    action_enable = enable;
  }

  bool GetActionControl()
  {
    return action_enable;
  }
public:
  const std::string CONFIG_DIR = "/home/mi/.cyberdog/audio";
  const std::string CONFIG_FILE = "settings.json";

private:
  /* data */
  std::shared_ptr<CyberdogAction> ptr_action;
  std::thread t_spin;
  bool action_enable;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__VOICE_CONTROL_HPP_
