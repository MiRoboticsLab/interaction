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
#ifndef CYBERDOG_FACE__CYBERDOG_FACE_ENTRY_HPP_
#define CYBERDOG_FACE__CYBERDOG_FACE_ENTRY_HPP_
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <string>
#include <memory>
#include <map>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/face_manager.hpp"
#include "protocol/srv/face_entry.hpp"
#include "protocol/srv/face_rec.hpp"
#include "protocol/srv/algo_manager.hpp"
#include "protocol/msg/face_result.hpp"
#include "protocol/msg/body.hpp"
#include "protocol/msg/face.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/face_info.hpp"
#include "protocol/msg/algo_list.hpp"
#include "protocol/msg/person.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/face_recognition_result.hpp"
#include "protocol/msg/face_entry_result.hpp"
#include "user_info_manager/UserAccountManager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cyberdog
{
namespace interaction
{

class face
{
  using VisionFaceEntrySrv = protocol::srv::FaceManager;
  using VisionFaceEntryMsg = protocol::msg::FaceResult;
  using FaceEntrySrv = protocol::srv::FaceEntry;
  using VisionSrv = protocol::srv::AlgoManager;
  using VisionMsg = protocol::msg::Person;
  using AlgoMsg = protocol::msg::AlgoList;
  using FaceRecSrv = protocol::srv::FaceRec;
  using AudioMsg = protocol::msg::AudioPlay;
  using FaceRecResultMsg = protocol::msg::FaceRecognitionResult;
  using FaceEntryResultMsg = protocol::msg::FaceEntryResult;
  using GetState = lifecycle_msgs::msg::State;
  using ChangeState = lifecycle_msgs::msg::Transition;
  using SwitchState = enum
  {
    RESULT_SUCCESS = 0,
    RESULT_TIMEOUT = 3,
    RESULT_NO_FACE_FOUND = 7,
    RESULT_MULTI_FACE_FOUND = 8,
    RESULT_KEEP_STABLE = 9,
    RESULT_DEGREE_NOT_SATISFIED = 10,
    RESULT_DISTANCE_NOT_SATISFIED = 11,
    RESULT_DEGREE_HEAD_LEFT = 12,
    RESULT_DEGREE_HEAD_RIGHT = 13,
    RESULT_DEGREE_HEAD_DOWN = 14,
    RESULT_DEGREE_HEAD_UP = 15,
    RESULT_DEGREE_HEAD_TILT = 16,
    RESULT_FACE_ALREADY_EXIST = 17
  };

  struct AlgoManagerRequest
  {
    bool open_age;
    bool open_emotion;
    std::vector<uint8_t> algo_enable;
    std::vector<uint8_t> algo_disable;
    AlgoManagerRequest()
    {
      open_age = false;
      open_emotion = false;
    }
  };

public:
  explicit face(const std::string & name);
  bool Init();
  void Run();
  ~face();

private:
  bool face_recognition_life_cycle_ {false};  // 人脸识别生命周期，True为有效
  const double FACE_ENTRY_TIME_INTERVAL = 6.0;
  int32_t FACE_RECOGNITION_TIME_INTERVAL;
  int32_t flag_;
  bool wake_face_recognition_call = false;
  time_t face_entry_time_ = time(nullptr);
  time_t face_recognition_time_ = time(nullptr);
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::mutex face_rec_mtx;
  std::map<
    std::string /* name */,
    std::map<
      std::string /* request_id */,
      std::chrono::time_point<std::chrono::system_clock /* timeout */>>>
  add_recognition_map;    /* 用户合法队列 */

  rclcpp::TimerBase::SharedPtr update_timer_ {nullptr};              // [定时器]更新数据
  // entry
  std::shared_ptr<VisionFaceEntrySrv::Request> ai_face_entry_request_ {nullptr};
  std::shared_ptr<VisionFaceEntrySrv::Request> ai_face_add_face_request_ {nullptr};

  rclcpp::Service<FaceEntrySrv>::SharedPtr grpc_face_entry_service_ {nullptr};
  rclcpp::Client<VisionFaceEntrySrv>::SharedPtr ai_face_entry_client_ {nullptr};
  rclcpp::Subscription<VisionFaceEntryMsg>::SharedPtr face_entry_sub_ {nullptr};
  rclcpp::Publisher<FaceEntryResultMsg>::SharedPtr face_entry_result_pub_ {nullptr};
  // recognition
  std::shared_ptr<VisionSrv::Request> ai_face_recognition_request_ {nullptr};
  rclcpp::Service<FaceRecSrv>::SharedPtr grpc_face_recognition_service_ {nullptr};
  rclcpp::Client<VisionSrv>::SharedPtr ai_face_recognition_client_ {nullptr};
  rclcpp::Subscription<VisionMsg>::SharedPtr face_recognition_sub_ {nullptr};
  std::shared_ptr<FaceRecSrv::Request> grpc_face_recognition_request_ {nullptr};
  rclcpp::Publisher<FaceRecResultMsg>::SharedPtr face_recognition_result_pub_ {nullptr};
  // 数据修改
  cyberdog::common::CyberdogAccountManager obj;
  // 激活ai服务
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> ai_get_state_ {nullptr};
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> ai_change_state_ {nullptr};
  // audio
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_pub_ {nullptr};
  std::shared_ptr<AudioMsg> face_audiomsg_ptr_;
  // 创建回调函数组
  rclcpp::CallbackGroup::SharedPtr face_callback_group_;
  rclcpp::CallbackGroup::SharedPtr time_callback_group_;
  rclcpp::CallbackGroup::SharedPtr face_entry_callback_group_;
  rclcpp::CallbackGroup::SharedPtr face_recognition_callback_group_;
  void UpdateStatus();                  // 更新状态
  bool Is_Active();
  bool Is_Deactive();
  void Face_Entry_Proccess_Fun(
    const std::shared_ptr<FaceEntrySrv::Request> request,
    std::shared_ptr<FaceEntrySrv::Response> response);
  void Face_Entry_Result_Callback(VisionFaceEntryMsg mgs);

  void Face_Recognition_Proccess_Fun(
    const std::shared_ptr<FaceRecSrv::Request> request,
    std::shared_ptr<FaceRecSrv::Response> response);
  void Face_Recognition_Result_Callback(VisionMsg mgs);
  void Face_Recognition_Request_Load(AlgoManagerRequest face_rec_request);
};  // class face
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_FACE__CYBERDOG_FACE_ENTRY_HPP_
