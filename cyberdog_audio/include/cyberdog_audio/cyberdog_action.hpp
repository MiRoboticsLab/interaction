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
#ifndef CYBERDOG_AUDIO__CYBERDOG_ACTION_HPP_
#define CYBERDOG_AUDIO__CYBERDOG_ACTION_HPP_

#include <string>
#include <map>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_utils/motion_utils.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "cyberdog_audio/follow_me.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/bms_status.hpp"

// using std::chrono_literals::operator""ms;

namespace cyberdog
{
namespace interaction
{
enum class DogAction : uint8_t
{
  ACTION_STANDING = 0,  // 站立
  ACTION_BACK,  // 后退
  ACTION_DANCE,  // 跳舞
  ACTION_BALLET,  // 芭蕾舞
  ACTION_COME,  // 过来
  ACTION_GO_ROUND,  // 原地转圈
  ACTION_HIGH_FIVE,  // 击掌
  ACTION_BACK_SOMERSAULT,  // 后空翻
  ACTION_MOVE_FORWARD,  // 往前走
  ACTION_FOLLOW_ME,  // 跟我走
  ACTION_CHANGE_HANDS,  // 换个手
  ACTION_SIT_DOWN,  // 坐下
  ACTION_WAGGING_TAIL,  // 摇尾巴
  ACTION_SHAKE_HEAD,  // 摇头
  ACTION_NOD,  // 点头
  ACTION_BOW,  // 鞠躬
  ACTION_PLAY_DEAD,  // 装死
  ACTION_GO_SLEEP,  // 去睡觉
  ACTION_SPACE_WALK,  // 太空步
  ACTION_JUMP,  // 跳跃
  ACTION_DOWN,  // 趴下
  ACTION_UNKOWN,  // 未知
  ACTION_RIGHT_HAND,  // 握右手
  ACTION_LEFT_HAND,  // 握左手
  ACTION_AND_LAZY,  // 伸懒腰
};  // enum class DogAction

enum class DogEmotion : uint8_t
{
  EMOTION_NORMAL = 0,  // 无表情
  EMOTION_JOYFUL,  // 喜悦
  EMOTION_SAD,  // 悲伤
  EMOTION_STUCK_ON,  // 迷恋
  EMOTION_EXCITED,  // 激动
  EMOTION_ANXIETY,  // 焦虑
  EMOTION_EXPECT,  // 期待
  EMOTION_PROUND,  // 自豪
  EMOTION_SHY,  // 害羞
  EMOTION_HAPPY,  // 开心
  EMOTION_NAUGHTY,  // 调皮
  EMOTION_CONFUSED,  // 迷茫
  EMOTION_AMUSE,  // 逗乐
  EMOTION_DELIGHTED,  // 欢腾
  EMOTION_WORRY,  // 担忧
  EMOTION_EMBARRASS,  // 尴尬
  EMOTION_THANKFUL,  // 感恩
  EMOTION_CALM,  // 平静
  // 难过
};

using DOG_ACTION_CALLBACK = std::function<void (const uint8_t &)>;

class CyberdogAction final : public rclcpp::Node
{
public:
  CyberdogAction(/* args */)
  : Node("audio_action")
  {
    utc_node_ptr_ = std::make_shared<Uwb_Tracking_Client>();
    std::thread(
      [this]() {
        rclcpp::spin(utc_node_ptr_);
      }).detach();
    motion_status_sub_ =
      this->create_subscription<protocol::msg::MotionStatus>(
      "motion_status", 10,
      std::bind(&CyberdogAction::GetCurrentMotionID, this, std::placeholders::_1));
    motion_servo_request_pub_ =
      this->create_publisher<protocol::msg::MotionServoCmd>(
      "motion_servo_cmd", rclcpp::SystemDefaultsQoS());
    audio_play_pub =
      this->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS());
    motion_ressult_client_ =
      this->create_client<protocol::srv::MotionResultCmd>("motion_result_cmd");
    bms_status_sub_ = 
       this->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", 10,
      std::bind(&CyberdogAction::BmsStatus, this, std::placeholders::_1));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_UNKOWN,
        std::bind(&CyberdogAction::Unkown, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_STANDING,
        std::bind(&CyberdogAction::Standing, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_BACK,
        std::bind(&CyberdogAction::Back, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_DANCE,
        std::bind(&CyberdogAction::Dance, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_BALLET,
        std::bind(&CyberdogAction::Ballet, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_COME,
        std::bind(&CyberdogAction::Come, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_GO_ROUND,
        std::bind(&CyberdogAction::Go_Round, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_HIGH_FIVE,
        std::bind(&CyberdogAction::High_Five, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_BACK_SOMERSAULT,
        std::bind(&CyberdogAction::Back_SomeResult, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_MOVE_FORWARD,
        std::bind(&CyberdogAction::Move_Forward, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_FOLLOW_ME,
        std::bind(&CyberdogAction::Follow_Me, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_CHANGE_HANDS,
        std::bind(&CyberdogAction::Change_Hands, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_SIT_DOWN,
        std::bind(&CyberdogAction::Sit_Down, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_WAGGING_TAIL,
        std::bind(&CyberdogAction::Wagging_Tail, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_SHAKE_HEAD,
        std::bind(&CyberdogAction::Shake_Head, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_NOD,
        std::bind(&CyberdogAction::Nod, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_BOW,
        std::bind(&CyberdogAction::Bow, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_PLAY_DEAD,
        std::bind(&CyberdogAction::Play_Dead, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_GO_SLEEP,
        std::bind(&CyberdogAction::Go_Sleep, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_SPACE_WALK,
        std::bind(&CyberdogAction::Space_Walk, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_JUMP,
        std::bind(&CyberdogAction::Jump, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_DOWN,
        std::bind(&CyberdogAction::Down, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_RIGHT_HAND,
        std::bind(&CyberdogAction::RIGHT_HAND, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_LEFT_HAND,
        std::bind(&CyberdogAction::LEFT_HAND, this, std::placeholders::_1)));
    execute_dog_action_map.insert(
      std::make_pair(
        DogAction::ACTION_AND_LAZY,
        std::bind(&CyberdogAction::AND_LAZY, this, std::placeholders::_1)));
  }
  ~CyberdogAction() = default;
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    action_enable = msg->power_wired_charging || msg->power_wp_charging;
  }
  void ExecuteAction(const std::string & action, const uint8_t & count)
  {
    INFO("execute action:%s", action.c_str());
    (void) count;
    INFO("action_enable:%d",static_cast<int>(action_enable));
    if(action_enable){
      INFO("充电中，不响应垂域指令控制");
      protocol::msg::AudioPlayExtend msg;
      msg.is_online = true;
      msg.module_name = "audio_action";
      msg.text = "充电中，无法控制设备，请断开电源后再试";
      audio_play_pub->publish(msg);
      return;
    }
    auto control_iter = control_dog_action_map.find(action);
    if (control_iter == control_dog_action_map.end()) {
      WARN("new action:%s", action.c_str());
      return;
    }
    auto execute_iter = execute_dog_action_map.find(
      control_dog_action_map.at(action));
    if (execute_iter == execute_dog_action_map.end()) {
      WARN("unimplement action:%s", action.c_str());
      return;
    }
    execute_iter->second(count);
  }

  void ExecuteSpecialAction(
    const std::string & action, const uint8_t & count,
    const std::string & dialog_id, std::string & emotion)
  {
    ExecuteAction(action, count);
    (void) dialog_id;
    (void) emotion;
  }

  void WakeUp(float direction)
  {
    (void) direction;
    utc_node_ptr_->cancel_goal();
  }

private:
  void GetCurrentMotionID(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    motion_id_ = msg->motion_id;
  }
  void Unkown(const uint8_t & times)
  {
    (void) times;
  }
  void RIGHT_HAND(const uint8_t & times)
  {
    (void) times;
    INFO("enter RIGHT_HAND");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 142;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void LEFT_HAND(const uint8_t & times)
  {
    (void) times;
    INFO("enter LEFT_HAND");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 141;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void AND_LAZY(const uint8_t & times)
  {
    (void) times;
    INFO("enter AND_LAZY");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 146;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Standing(const uint8_t & times)
  {
    (void) times;
    INFO("enter Standing");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 111;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Back(const uint8_t & times)
  {
    cyberdog::motion::MotionUtils & mu = cyberdog::motion::MotionUtils::GetMotionUtils();
    auto result = mu.ExecuteWalkDuration(1000 * times, -0.1, 0.0, 0.0);
    if (!result) {
      INFO("back error");
    }
  }
  void Dance(const uint8_t & times)
  {
    (void) times;
    INFO("enter Dance");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 140;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Ballet(const uint8_t & times)
  {
    (void) times;
    INFO("enter Ballet");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 151;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Come(const uint8_t & times)
  {
    (void) times;
    INFO("enter Come");
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = "audio_action";
    msg.text = "功能正在开发中，请等待";
    audio_play_pub->publish(msg);
  }
  void Go_Round(const uint8_t & times)
  {
    (void) times;
    INFO("enter Go_Round");
    cyberdog::motion::MotionUtils & mu = cyberdog::motion::MotionUtils::GetMotionUtils();
    auto result = mu.ExecuteWalkDuration(10000 * times, 0.0, 0.0, 0.3);
    if (!result) {
      INFO("back error");
    }
  }
  void High_Five(const uint8_t & times)
  {
    (void) times;
    INFO("enter High_Five");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 141;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Back_SomeResult(const uint8_t & times)
  {
    (void) times;
    INFO("enter Back_SomeResult");
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = "audio_action";
    msg.text = "主人，这个动作太危险了呢";
    audio_play_pub->publish(msg);
  }
  void Move_Forward(const uint8_t & times)
  {
    INFO("enter Move_Forward");
    cyberdog::motion::MotionUtils & mu = cyberdog::motion::MotionUtils::GetMotionUtils();
    auto result = mu.ExecuteWalkDuration(1000 * times, 0.1, 0.0, 0.0);
    if (!result) {
      INFO("move forward result error");
    }
  }
  void Follow_Me(const uint8_t & times)
  {
    if (utc_node_ptr_->get_uwb_device() != 0) {
      utc_node_ptr_->send_goal();
    } else {
      protocol::msg::AudioPlayExtend msg;
      msg.is_online = true;
      msg.module_name = "audio_action";
      msg.text = "暂无可跟随标签，请添加配件后再试";
      audio_play_pub->publish(msg);
    }
    (void) times;
  }
  void Change_Hands(const uint8_t & times)
  {
    (void) times;
    INFO("enter Change_Hands");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    if (motion_id_ == 142) {
      req->motion_id = 141;
    } else {
      req->motion_id = 142;
    }
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
    while (rsp.code == 3011) {
      rclcpp::WallRate loop_rate(10.0);
      loop_rate.sleep();
      INFO("miton is busy, try again");
      callMotionServoCmd(req, rsp);
    }
  }
  void Sit_Down(const uint8_t & times)
  {
    (void) times;
    INFO("enter Sit_Down");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 143;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Wagging_Tail(const uint8_t & times)
  {
    (void) times;
    INFO("enter Wagging_Tail");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 144;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Shake_Head(const uint8_t & times)
  {
    (void) times;
    INFO("enter Shake_Head");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 145;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Nod(const uint8_t & times)
  {
    (void) times;
    INFO("enter Nod");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 161;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Bow(const uint8_t & times)
  {
    (void) times;
    INFO("enter Bow");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 123;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Play_Dead(const uint8_t & times)
  {
    (void) times;
    INFO("enter Play_Dead");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 0;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Go_Sleep(const uint8_t & times)
  {
    (void) times;
    INFO("enter Go_Sleep");
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = "audio_action";
    msg.text = "主人，我还不困呢";
    audio_play_pub->publish(msg);
  }
  void Space_Walk(const uint8_t & times)
  {
    (void) times;
    INFO("enter Space_Walk");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 152;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Jump(const uint8_t & times)
  {
    (void) times;
    INFO("enter Jump");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 136;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }
  void Down(const uint8_t & times)
  {
    (void) times;
    INFO("enter Down");
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp;
    req->motion_id = 101;
    callMotionServoCmd(req, rsp);
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
  }

  void callMotionServoCmd(
    const std::shared_ptr<protocol::srv::MotionResultCmd::Request> req,
    protocol::srv::MotionResultCmd::Response & rsp)
  {
    std::chrono::seconds timeout(15);
    DEBUG("callMotionServoCmd motion_id: %d.", req->motion_id);
    INFO("callMotionServoCmd motion_id: %d.", req->motion_id);
    req->cmd_source = 1;
    auto future_result = motion_ressult_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      DEBUG("success to call callMotionServoCmd services.");
      INFO("success to call callMotionServoCmd services.");
    } else {
      INFO("Failed to call callMotionServoCmd services.");
      rsp.code = -1;
      return;
    }
    rsp.motion_id = future_result.get()->motion_id;
    rsp.result = future_result.get()->result;
    rsp.code = future_result.get()->code;
  }
private:
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_pub;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr motion_servo_request_pub_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_;
  int motion_id_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_ressult_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Uwb_Tracking_Client> utc_node_ptr_;
  /* data */
  const std::map<std::string, DogAction> control_dog_action_map = {
    {"UNKOWN", DogAction::ACTION_UNKOWN},
    {"STANDING", DogAction::ACTION_STANDING},
    {"BACK", DogAction::ACTION_BACK},
    {"DANCE", DogAction::ACTION_DANCE},
    {"BALLET", DogAction::ACTION_BALLET},
    {"COME", DogAction::ACTION_COME},
    {"GO_ROUND", DogAction::ACTION_GO_ROUND},
    {"HIGH_FIVE", DogAction::ACTION_HIGH_FIVE},
    {"BACK_SOMERSAULT", DogAction::ACTION_BACK_SOMERSAULT},
    {"MOVE_FORWARD", DogAction::ACTION_MOVE_FORWARD},
    {"FOLLOW_ME", DogAction::ACTION_FOLLOW_ME},
    {"CHANGE_HANDS", DogAction::ACTION_CHANGE_HANDS},
    {"SIT_DOWN", DogAction::ACTION_SIT_DOWN},
    {"WAGGING_TAIL", DogAction::ACTION_WAGGING_TAIL},
    {"SHAKE_HEAD", DogAction::ACTION_SHAKE_HEAD},
    {"NOD", DogAction::ACTION_NOD},
    {"BOW", DogAction::ACTION_BOW},
    {"PLAY_DEAD", DogAction::ACTION_PLAY_DEAD},
    {"GO_SLEEP", DogAction::ACTION_GO_SLEEP},
    {"SPACE_WALK", DogAction::ACTION_SPACE_WALK},
    {"JUMP", DogAction::ACTION_JUMP},
    {"DOWN", DogAction::ACTION_DOWN},
    {"RIGHT_HAND", DogAction::ACTION_RIGHT_HAND},
    {"LEFT_HAND", DogAction::ACTION_LEFT_HAND},
    {"AND_LAZY", DogAction::ACTION_AND_LAZY},
  };
  const std::map<std::string, DogEmotion> control_dog_emotion_map = {
    {"UNKOWN", DogEmotion::EMOTION_NORMAL},
    {"JOYFUL", DogEmotion::EMOTION_JOYFUL},
    {"SAD", DogEmotion::EMOTION_SAD},
    {"STUCK_ON", DogEmotion::EMOTION_STUCK_ON},
    {"EXCITED", DogEmotion::EMOTION_EXCITED},
    {"ANXIETY", DogEmotion::EMOTION_ANXIETY},
    {"EXPECT", DogEmotion::EMOTION_EXPECT},
    {"PROUND", DogEmotion::EMOTION_PROUND},
    {"SHY", DogEmotion::EMOTION_SHY},
    {"HAPPY", DogEmotion::EMOTION_HAPPY},
    {"NAUGHTY", DogEmotion::EMOTION_NAUGHTY},
    {"CONFUSED", DogEmotion::EMOTION_CONFUSED},
    {"AMUSE", DogEmotion::EMOTION_AMUSE},
    {"DELIGHTED", DogEmotion::EMOTION_DELIGHTED},
    {"WORRY", DogEmotion::EMOTION_WORRY},
    {"EMBARRASS", DogEmotion::EMOTION_EMBARRASS},
    {"THANKFUL", DogEmotion::EMOTION_THANKFUL},
    {"CALM", DogEmotion::EMOTION_CALM},
  };
  std::map<DogAction, DOG_ACTION_CALLBACK> execute_dog_action_map;
  bool action_enable = false;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__CYBERDOG_ACTION_HPP_
