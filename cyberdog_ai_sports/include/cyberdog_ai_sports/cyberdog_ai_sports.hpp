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
#ifndef CYBERDOG_AI_SPORTS__CYBERDOG_AI_SPORTS_HPP_
#define CYBERDOG_AI_SPORTS__CYBERDOG_AI_SPORTS_HPP_
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "protocol/srv/algo_manager.hpp"
#include "protocol/msg/algo_list.hpp"
#include "protocol/msg/person.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/body.hpp"
#include "protocol/msg/keypoint.hpp"
#include "protocol/srv/sport_manager.hpp"
#include "protocol/msg/sport_counts_result.hpp"
#include "protocol/msg/audio_play.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "cyberdog_ai_sports/calculate_angle.hpp"
#include "cyberdog_ai_sports/sport_counts.hpp"

namespace cyberdog
{
namespace interaction
{

class sport final : public rclcpp::Node
{
  using VisionSrv = protocol::srv::AlgoManager;
  using VisionMsg = protocol::msg::Person;
  using AlgoMsg = protocol::msg::AlgoList;
  using AudioMsg = protocol::msg::AudioPlay;
  using StateMsg = lifecycle_msgs::msg::State;
  using TransMsg = lifecycle_msgs::msg::Transition;
  using GStateSrv = lifecycle_msgs::srv::GetState;
  using CStateSrv = lifecycle_msgs::srv::ChangeState;
  using SportSrv = protocol::srv::SportManager;
  using SportMsg = protocol::msg::SportCountsResult;

  struct AlgoManagerRequest
  {
    bool open_age_;
    bool open_emotion_;
    std::vector<uint8_t> algo_enable_;
    std::vector<uint8_t> algo_disable_;
    AlgoManagerRequest()
    {
      open_age_ = false;
      open_emotion_ = false;
    }
  };

public:
  explicit sport(const std::string & name);
  bool Init();
  void Run();
  ~sport();

private:
  std::string name_ {""};                                                   // node 名称
  std::string params_pkg_dir_ {""};                                         // params 包路径
  std::string node_config_dir_ {""};                                        // node 配置文件路径
  toml::value params_toml_;
  int32_t sport_counts_inter_time_ = 300;                                    // 默认算法启停时长300s
  uint8_t sport_type_;                                                       // 运动类型
  int32_t sport_counts_;                                                     // 运动个数
  int32_t update_counts_;
  int32_t start_time_;                                                       // 算法开始时间
  int32_t end_time_;                                                         // 算法停止时间
  int32_t flag_ = 0;                                                         // 标志位
  std::mutex sport_counts_mtx_;
  std::vector<float> calculate_Angle1_;
  std::vector<float> calculate_Angle2_;
  std::vector<float> calculate_Angle3_;
  bool wake_sport_counts_call_ = false;
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::TimerBase::SharedPtr update_timer_ {nullptr};                     // [定时器]检查是否超时
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_pub_ {nullptr};
  std::shared_ptr<AudioMsg> sport_audiomsg_ptr_;
  // 运动计数
  std::shared_ptr<VisionSrv::Request> ai_sports_request_{nullptr};
  rclcpp::Service<SportSrv>::SharedPtr sport_counts_service_{nullptr};
  rclcpp::Client<VisionSrv>::SharedPtr ai_sports_client_{nullptr};
  rclcpp::Subscription<VisionMsg>::SharedPtr ai_sports_sub_{nullptr};
  rclcpp::Publisher<SportMsg>::SharedPtr sport_counts_result_pub_{nullptr};
  // 激活ai服务
  std::shared_ptr<rclcpp::Client<GStateSrv>> ai_get_state_ {nullptr};
  std::shared_ptr<rclcpp::Client<CStateSrv>> ai_change_state_ {nullptr};
  // 创建回调函数组
  rclcpp::CallbackGroup::SharedPtr ctl_life_cb_group_{nullptr};
  rclcpp::CallbackGroup::SharedPtr ai_sport_cb_group_{nullptr};
  rclcpp::CallbackGroup::SharedPtr sport_counts_cb_group_{nullptr};
  rclcpp::CallbackGroup::SharedPtr time_cb_group_{nullptr};

private:
  void ResetAlgo();
  bool ReadTomlConfig();
  bool Is_Configure();
  bool Is_Active();
  bool Is_Deactive();
  void UpdateStatus();
  int  ProcessKeypoint(
    std::vector<std::vector<XMPoint>> & multHuman,
    uint32_t & height, uint32_t & width);
  void Ai_Sport_Process_Fun(
    const std::shared_ptr<SportSrv::Request> request,
    std::shared_ptr<SportSrv::Response> response);
  void Ai_Sport_Result_Callback(VisionMsg msg);
  void Ai_Sport_Request_Load(AlgoManagerRequest sport_counts_request);
  int squat_dataFusion_;
  int squat_dataFiler_;
  int squat_threMinAngle_;
  int squat_threMaxAngle_;
  int highKnee_dataFusion_;
  int highKnee_dataFiler_;
  int highKnee_threMinAngle_;
  int highKnee_threMaxAngle_;
  int sitUp_dataFusion_;
  int sitUp_dataFiler_;
  int sitUp_threMinAngle_;
  int sitUp_threMidAngle1_;
  int sitUp_threMidAngle2_;
  int sitUp_threMaxAngle_;
  int pressUp_dataFusion_;
  int pressUp_dataFiler_;
  int pressUp_threMaxAngle_;
  int pressUp_difMinAngle_;
  int pressUp_difMaxAngle_;
  int plank_dataFusion_;
  int plank_dataFiler_;
  int plank_threMinAngle_;
  int plank_threMaxAngle_;
  int jump_dataFusion_;
  int jump_dataFiler_;
  int jump_threMidAngle1_;
  int jump_threMidAngle2_;
  int jump_threMidAngle3_;
  int jump_threMidAngle4_;
  int jump_threMidAngle5_;
  int jump_threMidAngle6_;
  int jump_threMidAngle7_;
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AI_SPORTS__CYBERDOG_AI_SPORTS_HPP_
