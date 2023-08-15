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
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_face/cyberdog_face_entry.hpp"


namespace cyberdog
{
namespace interaction
{

face::face(const std::string & name)
{
  this->node_ptr_ = rclcpp::Node::make_shared(name);
  INFO("Creating cyberdog_face object(node)");
}
face::~face()
{
  INFO("Destroy [face] object(node)");
}

bool face::Init()
{
  // create callbackgroup
  face_callback_group_ = this->node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  time_callback_group_ = this->node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  face_entry_callback_group_ = this->node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  face_recognition_callback_group_ = this->node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = face_callback_group_;
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = face_callback_group_;
  // 激活ai_lifecycle
  this->ai_change_state_ = this->node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>(
    "vision_manager/change_state", rmw_qos_profile_services_default, time_callback_group_);
  this->ai_get_state_ = this->node_ptr_->create_client<lifecycle_msgs::srv::GetState>(
    "vision_manager/get_state", rmw_qos_profile_services_default, time_callback_group_);
  // 延时等待vision启动
  while (!this->ai_get_state_->wait_for_service(std::chrono::seconds(1))) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  INFO("vision_manager already start.");
  std::this_thread::sleep_for(std::chrono::milliseconds(30000));
  auto get_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.wait_for(std::chrono::seconds(100)) != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(1))) {
    ERROR("ai_change_state service not ready.");
    return false;
  }
  if (get_respone.get()->current_state.id == GetState::PRIMARY_STATE_UNCONFIGURED) {
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = ChangeState::TRANSITION_CONFIGURE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(150)) != std::future_status::ready) {
      ERROR("Failed to call service /change_state");
    } else {
      if (change_respone.get()->success) {
        INFO("Transition active successfully triggered.");
      } else {
        WARN("Failed to transform");
      }
    }
  }
  INFO("face node init");
  // face entry init
  this->grpc_face_entry_service_ = this->node_ptr_->create_service<FaceEntrySrv>(
    "cyberdog_face_entry_srv",
    std::bind(&face::Face_Entry_Proccess_Fun, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, face_callback_group_);
  this->ai_face_entry_client_ = this->node_ptr_->create_client<VisionFaceEntrySrv>(
    "facemanager", rmw_qos_profile_services_default, face_entry_callback_group_);
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  // create subscribe
  this->face_entry_sub_ = this->node_ptr_->create_subscription<VisionFaceEntryMsg>(
    "facemanager/face_result", pub_qos,
    std::bind(&face::Face_Entry_Result_Callback, this, std::placeholders::_1),
    sub_options);
  this->face_entry_result_pub_ = this->node_ptr_->create_publisher<FaceEntryResultMsg>(
    "face_entry_msg", rclcpp::SystemDefaultsQoS(), pub_options);
  this->ai_face_entry_request_ = std::make_shared<VisionFaceEntrySrv::Request>();
  this->ai_face_add_face_request_ = std::make_shared<VisionFaceEntrySrv::Request>();

  // face recognition
  this->grpc_face_recognition_service_ = this->node_ptr_->create_service<FaceRecSrv>(
    "cyberdog_face_recognition_srv",
    std::bind(
      &face::Face_Recognition_Proccess_Fun, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, face_callback_group_);
  this->ai_face_recognition_client_ = this->node_ptr_->create_client<VisionSrv>(
    "algo_manager", rmw_qos_profile_services_default, face_recognition_callback_group_);
  this->ai_face_recognition_request_ = std::make_shared<VisionSrv::Request>();
  this->grpc_face_recognition_request_ = std::make_shared<FaceRecSrv::Request>();
  this->face_recognition_result_pub_ = this->node_ptr_->create_publisher<FaceRecResultMsg>(
    "face_rec_msg", rclcpp::SystemDefaultsQoS(), pub_options);
  // audio
  this->audio_pub_ = this->node_ptr_->create_publisher<AudioMsg>("speech_play", 10, pub_options);
  this->face_audiomsg_ptr_ = std::make_shared<AudioMsg>();
  this->face_audiomsg_ptr_->module_name = "face";
  // create Timer
  this->update_timer_ = this->node_ptr_->create_wall_timer(
    std::chrono::nanoseconds(static_cast<int64_t>(1000000000 / 1)),
    std::bind(&face::UpdateStatus, this), time_callback_group_);
  return true;
}

void face::UpdateStatus()
{
  std::unique_lock<std::mutex> lg_face_map(face_rec_mtx);
  for (auto name_iter = this->add_recognition_map.begin();
    name_iter != add_recognition_map.end(); )
  {
    for (auto id_iter = name_iter->second.begin(); id_iter != name_iter->second.end(); ) {
      if (std::chrono::system_clock::now() > id_iter->second) {
        id_iter = name_iter->second.erase(id_iter);
      } else {
        id_iter++;
      }
    }
    if (name_iter->second.empty() && !this->add_recognition_map.empty()) {
      name_iter = this->add_recognition_map.erase(name_iter);
    } else {
      name_iter++;
    }
  }
  if (add_recognition_map.empty()) {
    this->flag_ = 0;
  }
}

void face::Face_Recognition_Result_Callback(VisionMsg msg)
{
  if (this->wake_face_recognition_call == false) {
    return;
  }
  INFO("get face recognition info topic .");
  if (add_recognition_map.empty()) {
    INFO("Turn off face recognition callback.");
    this->wake_face_recognition_call = false;
    this->face_recognition_life_cycle_ = false;
    this->face_recognition_sub_ = nullptr;
    this->flag_ = 0;
    // ai模式置为inactive
    if (Is_Deactive()) {
      INFO("Successfully set AI model to Inactive");
    }
    return;
  }
  if (msg.face_info.count > 0) {
    INFO("detect %d face .", static_cast<int>(msg.face_info.count));
    for (auto single_face_info : msg.face_info.infos) {
      std::lock_guard<std::mutex> lg_face_map(face_rec_mtx);
      auto iter = add_recognition_map.find(single_face_info.id);
      bool case1 = (this->add_recognition_map.find("All") != this->add_recognition_map.end()) &&
        single_face_info.id != "stranger" && single_face_info.id != "jushi";
      bool case2 = single_face_info.id == iter->first;
      if (case1) {
        INFO("delete All.");
        auto name_iter = this->add_recognition_map.find("All");
        auto id_iter = name_iter->second.begin();
        std::string rec_id = id_iter->first;
        INFO("the value of Rec_id: %s.", rec_id.c_str());
        name_iter->second.erase(id_iter);
        if (name_iter->second.empty()) {
          this->add_recognition_map.erase(add_recognition_map.find("All"));
        }
        INFO("check the number of maps: %d.", static_cast<int>(this->add_recognition_map.size()));
        std::shared_ptr<FaceRecResultMsg> face_recoginition_msg_ptr_ =
          std::make_shared<FaceRecResultMsg>();
        face_recoginition_msg_ptr_->result = FaceRecResultMsg::RESULT_SUCCESS;
        face_recoginition_msg_ptr_->username = single_face_info.id;
        face_recoginition_msg_ptr_->id = rec_id;
        face_recoginition_msg_ptr_->age = single_face_info.age;
        face_recoginition_msg_ptr_->emotion = single_face_info.emotion;
        this->face_recognition_result_pub_->publish(*face_recoginition_msg_ptr_);
        INFO(
          "recognized face username %s, face id: %s.",
          single_face_info.id.c_str(), rec_id.c_str());
        INFO("recognized age %f, emotion: %d.", single_face_info.age, single_face_info.emotion);
        INFO("face recognition successfully.");
        // 播报人脸识别成功语音
        INFO("Broadcast face recognition success voice.");
        // 人脸识别成功
        this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_RECGONITION_FINISH;
        this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
      }
      if (case2) {
        INFO("delete username.");
        std::string queue_id;   // NOLINT
        if (iter != this->add_recognition_map.end()) {
          queue_id = iter->second.begin()->first;
          iter->second.erase(iter->second.begin());
          if (iter->second.empty()) {
            add_recognition_map.erase(iter);
          }
          INFO("check the number of maps: %d.", static_cast<int>(add_recognition_map.size()));
        }
        std::shared_ptr<FaceRecResultMsg> face_recoginition_msg_ptr_ =
          std::make_shared<FaceRecResultMsg>();
        face_recoginition_msg_ptr_->result = FaceRecResultMsg::RESULT_SUCCESS;
        face_recoginition_msg_ptr_->username = single_face_info.id;
        face_recoginition_msg_ptr_->id = queue_id;
        face_recoginition_msg_ptr_->age = single_face_info.age;
        face_recoginition_msg_ptr_->emotion = single_face_info.emotion;
        this->face_recognition_result_pub_->publish(*face_recoginition_msg_ptr_);
        INFO(
          "recognized face username %s, face id: %s.",
          single_face_info.id.c_str(), queue_id.c_str());
        INFO("recognized age %f,emotion: %d.", single_face_info.age, single_face_info.emotion);
        INFO("face recognition successfully.");
        // 播报人脸识别成功语音
        INFO("Broadcast face recognition success voice.");
        // 人脸识别成功
        this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_RECGONITION_FINISH;
        this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
      }
      if (case1 == false && case2 == false) {
        INFO("detected face not in face feature file.");
      }
    }
  } else {
    INFO("can not detect face.");
  }
}

void face::Face_Recognition_Request_Load(AlgoManagerRequest face_rec_request)
{
  this->ai_face_recognition_request_->open_age = face_rec_request.open_age;
  this->ai_face_recognition_request_->open_emotion = face_rec_request.open_age;
  AlgoMsg algo_enable_list;
  for (auto algo_enable_single : face_rec_request.algo_enable) {
    algo_enable_list.algo_module = algo_enable_single;
    this->ai_face_recognition_request_->algo_enable.push_back(algo_enable_list);
    INFO(
      "need enable algo %d",
      static_cast<int>(algo_enable_list.algo_module));
  }
  AlgoMsg algo_disable_list;
  for (auto algo_disable_single : face_rec_request.algo_disable) {
    algo_disable_list.algo_module = algo_disable_single;
    INFO(
      "need disable algo %d",
      static_cast<int>(algo_enable_list.algo_module));
    this->ai_face_recognition_request_->algo_disable.push_back(algo_disable_list);
  }
}

void face::Face_Recognition_Proccess_Fun(
  const std::shared_ptr<FaceRecSrv::Request> request,
  std::shared_ptr<FaceRecSrv::Response> response)
{
  if (this->flag_ == 1 && request->command !=
    FaceRecSrv::Request::COMMAND_RECOGNITION_CANCEL)
  {
    INFO("Face entry algorithm in use.");
    return;
  }
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = face_callback_group_;
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  this->face_recognition_sub_ = this->node_ptr_->create_subscription<VisionMsg>(
    "person", pub_qos,
    std::bind(&face::Face_Recognition_Result_Callback, this, std::placeholders::_1), sub_options);
  auto set_time = [&]() -> bool {   // 设置时间
      // 设置人脸识别时间
      if (request->timeout < 0) {
        FACE_RECOGNITION_TIME_INTERVAL = FaceRecSrv::Request::DEFAULT_TIMEOUT;
      } else if (request->timeout > FaceRecSrv::Request::MAX_TIMEOUT) {
        FACE_RECOGNITION_TIME_INTERVAL = FaceRecSrv::Request::MAX_TIMEOUT;
      } else if (request->timeout < FaceRecSrv::Request::MIN_TIMEOUT) {
        FACE_RECOGNITION_TIME_INTERVAL = FaceRecSrv::Request::MIN_TIMEOUT;
      } else {
        FACE_RECOGNITION_TIME_INTERVAL = request->timeout;
      }
      return true;
    };

  auto update_recognition_map = [&]() -> bool {
      std::string target_name = (request->command == 0) ? "All" : request->username;
      auto name_ptr = this->add_recognition_map.find(target_name);
      if (name_ptr != this->add_recognition_map.end()) {
        auto request_ptr = name_ptr->second.find(request->id);
        if (request_ptr != name_ptr->second.end()) {
          if (request->command == 2) {
            this->add_recognition_map.clear();
            return true;
          } else if (request->command == 1) {
            return false;
          }
        } else {
          if (request->command == 2) {
            this->add_recognition_map.clear();
            return true;
          } else if ((request->command == 1) || (request->command == 0)) {
            name_ptr->second.insert(
              std::map<std::string,
              std::chrono::time_point<std::chrono::system_clock>>::value_type(
                request->id,
                std::chrono::system_clock::now() +
                std::chrono::seconds(FACE_RECOGNITION_TIME_INTERVAL)));
          }
        }
      } else {
        if (request->command == 2) {
          this->add_recognition_map.clear();
          return true;
        } else if ((request->command == 1) || (request->command == 0)) {
          std::map<
            std::string,
            std::chrono::time_point<std::chrono::system_clock>> new_meta;
          new_meta.insert(
            std::map<std::string,
            std::chrono::time_point<std::chrono::system_clock>>::value_type(
              request->id,
              std::chrono::system_clock::now() +
              std::chrono::seconds(FACE_RECOGNITION_TIME_INTERVAL)));
          this->add_recognition_map.insert(
            std::map<std::string,
            std::map<
              std::string,
              std::chrono::time_point<std::chrono::system_clock>>>::value_type(
              target_name,
              new_meta));
        }
        return true;
      }
    };
  auto start_AI = [&]() -> bool {
      if (request->command == FaceRecSrv::Request::COMMAND_RECOGNITION_CANCEL) {
        INFO("Cancel face recognition.");
        this->flag_ = 0;
        return true;
      }
      INFO("number of maps when requesting: %d.", static_cast<int>(add_recognition_map.size()));
      AlgoManagerRequest face_rec_request;
      face_rec_request.open_age = true;
      face_rec_request.open_emotion = true;
      face_rec_request.algo_enable = {AlgoMsg::ALGO_FACE};
      this->Face_Recognition_Request_Load(face_rec_request);
      INFO("Send a face recognition request to the ai module.");
      auto ai_response = this->ai_face_recognition_client_->async_send_request(
        this->ai_face_recognition_request_);
      auto status = ai_response.wait_for(std::chrono::seconds(10));
      if (status == std::future_status::ready) {
        this->flag_ = 1;
        Is_Active();
        INFO(
          "Receive the return response of the request to recognize face %d.",
          static_cast<int>(ai_response.get()->result_enable));
        response->result = static_cast<int>(ai_response.get()->result_enable);
        response->code = static_cast<int>(ai_response.get()->result_enable);
        if (this->ai_face_recognition_request_->algo_enable.size() > 0) {
          this->face_recognition_life_cycle_ = false;
          this->wake_face_recognition_call = true;
        }
        this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_RECOGNITION_REQUEST;
        this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
      } else {
        ERROR("Failed to call service /algo_manager");
        return false;
      }
      return true;
    };
  INFO("new cyberdog face recognition request comming.");
  if (set_time()) {
    if (update_recognition_map()) {
      if (start_AI() || (request->command == 2)) {
        response->result = FaceRecSrv::Response::ENABLE_SUCCESS;
        response->code = FaceRecSrv::Response::ENABLE_SUCCESS;
        return;
      }
    }
  }
  response->result = FaceRecSrv::Response::ENABLE_FAIL;
  response->code = FaceRecSrv::Response::ENABLE_FAIL;
}

void face::Face_Entry_Result_Callback(VisionFaceEntryMsg msg)
{
  // 设计语音提示触发机制
  if (msg.result == RESULT_SUCCESS) {
    // 向ai模块发送确认录入请求
    if (!this->ai_face_entry_client_->wait_for_service(std::chrono::seconds(1))) {
      ERROR("ai face service not ready.");
      return;
    }
    auto ai_response =
      this->ai_face_entry_client_->async_send_request(this->ai_face_add_face_request_);
    auto entry_rsp_status = ai_response.wait_for(std::chrono::seconds(2));
    if (entry_rsp_status == std::future_status::ready) {
      // 录入成功
      INFO(
        "Receive the return response of the request to confirm entry face %d",
        static_cast<int>(ai_response.get()->result));
      INFO(
        "Send face entry request conmand,%d,entry face %s",
        static_cast<int>(this->ai_face_add_face_request_->command),
        this->ai_face_add_face_request_->username.c_str());
      INFO("Face entered successfully.");
      this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FINISH;  // 录入成功
      INFO("Play the voice of successful face entry.");
      this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
      INFO("Publish face entry successful topic.");
      std::shared_ptr<FaceEntryResultMsg> face_entry_msg_ptr_ =
        std::make_shared<FaceEntryResultMsg>();
      face_entry_msg_ptr_->result = VisionFaceEntryMsg::RESULT_SUCCESS;
      face_entry_msg_ptr_->username = this->ai_face_add_face_request_->username;
      this->face_entry_result_pub_->publish(*face_entry_msg_ptr_);
      this->flag_ = 0;
      // 人脸数据录入
      auto add_result = obj.ModifyUserInformation(face_entry_msg_ptr_->username, 1, 1);
      INFO(
        "Return the result of adding face.%d",
        static_cast<int>(add_result));
      // ai模式置为inactive
      if (Is_Deactive()) {
        INFO("Successfully set AI model to Inactive");
      }
    }
  }
  if (msg.result == RESULT_FACE_ALREADY_EXIST) {
    // 录入失败，人脸已存在，请不要录入同一张脸
    INFO("Face entry failed.The face already exists,Please do not enter the same face.");
    this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ALREADY_EXIST;  // 录入超时，请重新录入
    INFO("Play the voice of face entry failed.");
    this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
    INFO("Publish face already exist topic.");
    std::shared_ptr<FaceEntryResultMsg> face_entry_msg_ptr_ =
      std::make_shared<FaceEntryResultMsg>();
    face_entry_msg_ptr_->result = VisionFaceEntryMsg::RESULT_FACE_ALREADY_EXIST;
    this->face_entry_result_pub_->publish(*face_entry_msg_ptr_);
    this->flag_ = 0;
    if (Is_Deactive()) {
      INFO("Successfully set AI model to Inactive");
    }
  }
  if (msg.result == RESULT_TIMEOUT) {
    // 录入超时，请重新录入
    INFO("Face entry timed out, please enter again.");
    this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_TIMEOUT;  // 录入超时，请重新录入
    INFO("Play the voice of face entry timeout.");
    this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
    INFO("Publish face entry timeout topic.");
    std::shared_ptr<FaceEntryResultMsg> face_entry_msg_ptr_ =
      std::make_shared<FaceEntryResultMsg>();
    face_entry_msg_ptr_->result = VisionFaceEntryMsg::RESULT_TIMEOUT;
    this->face_entry_result_pub_->publish(*face_entry_msg_ptr_);
    this->flag_ = 0;
    if (Is_Deactive()) {
      INFO("Successfully set AI model to Inactive");
    }
  }
  if (difftime(time(nullptr), this->face_entry_time_) >= this->FACE_ENTRY_TIME_INTERVAL) {
    this->face_entry_time_ = time(nullptr);
    switch (msg.result) {
      case RESULT_NO_FACE_FOUND: {
          WARN("No faces found in the image.");
          // 没有检测到人脸
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_NONE_FACES;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          break;
        }
      case RESULT_MULTI_FACE_FOUND: {
          // 发现多个人脸，请保持相机中图像中只有一个人脸～～～
          WARN("Multiple faces found, please keep only one face in the image.");
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_MUTIPLE_FACES;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          break;
        }
      case RESULT_KEEP_STABLE: {
          // 请保持稳定～～～
          WARN("please keep steady.");
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_STABLE;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          break;
        }
      case RESULT_DEGREE_NOT_SATISFIED: {
          // 请正对摄像头～～～
          WARN("please face the camera.");
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_POSE;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          break;
        }
      case RESULT_DISTANCE_NOT_SATISFIED: {
          // 请靠近摄像头～～～
          WARN("please get close to the camera.");
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_DISTANCE_CLOSE;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          break;
        }
      // 新增加
      case RESULT_DEGREE_HEAD_LEFT: {
          // 请向稍微左扭头，正对摄像头～～～由于图像和现实水平方向存在镜像，ai模块反馈的是以图像坐标系；
          // 语音提示以人为参考坐标系
          // 请向左扭头
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_POSE_LEFT;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          WARN("Please turn your head slightly to the left and face the camera.");
          break;
        }
      case RESULT_DEGREE_HEAD_RIGHT: {
          // 请向稍微右扭头，正对摄像头～～～
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_POSE_RIGHT;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          WARN("Please turn your head slightly to the right and face the camera.");
          break;
        }
      case RESULT_DEGREE_HEAD_DOWN: {
          // 请向稍微向下低头，正对摄像头～～～
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_POSE_DOWN;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          WARN("Please look down slightly and facing the camera.");
          break;
        }
      case RESULT_DEGREE_HEAD_UP: {
          // 请向稍微向上抬头，正对摄像头～～～
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_FIX_POSE_UP;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          WARN("Please look up slightly to face the camera.");
          break;
        }
      case RESULT_DEGREE_HEAD_TILT: {
          // 请不要歪曲，正对摄像头～～～
          this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_DEGREE_HEAD_TILT;
          this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
          WARN("Please do not tilt your head and face the camera");
          break;
        }
      default:
        break;
    }
  }
}
// 向app发送校验录入人脸图片；
void face::Face_Entry_Proccess_Fun(
  const std::shared_ptr<FaceEntrySrv::Request> request,
  std::shared_ptr<FaceEntrySrv::Response> response)
{
  if (this->flag_ == 1 && request->command == 0) {
    INFO("Face recognition algorithm in use.");
    response->result = FaceEntrySrv::Response::RESULT_BUSY;
    return;
  }
  try {
    INFO("get face entry request");
    INFO("command %d", request->command);
    this->ai_face_entry_request_->command = static_cast<uint8_t>(request->command);
    this->ai_face_entry_request_->ishost = request->ishost;
    this->ai_face_entry_request_->username = request->username;
    this->ai_face_entry_request_->oriname = request->oriname;
    if (request->command == 0) {
      AlgoManagerRequest face_rec_request;
      face_rec_request.algo_enable = {AlgoMsg::FACE_MANAGER};
      this->Face_Recognition_Request_Load(face_rec_request);
      auto face_entry_response = this->ai_face_recognition_client_->async_send_request(
        this->ai_face_recognition_request_);
      auto status = face_entry_response.wait_for(std::chrono::seconds(10));
      if (status == std::future_status::ready) {
        Is_Active();
      }
    }
    if (!this->ai_face_entry_client_->wait_for_service(std::chrono::seconds(5))) {
      ERROR("ai face service not ready");
      return;
    }
    auto ai_response =
      this->ai_face_entry_client_->async_send_request(this->ai_face_entry_request_);
    auto entry_rsp_status = ai_response.wait_for(std::chrono::seconds(10));
    if (entry_rsp_status == std::future_status::ready) {
      this->flag_ = 1;
      switch (request->command) {
        case FaceEntrySrv::Request::ADD_FACE: {
            this->ai_face_add_face_request_->command =
              VisionFaceEntrySrv::Request::CONFIRM_LAST_FACE;
            this->ai_face_add_face_request_->username = request->username;
            this->ai_face_add_face_request_->ishost = request->ishost;
            INFO("entry username %s face", request->username.c_str());
            INFO(
              "Receive the return response of the request to enter the face. %d",
              static_cast<int>(ai_response.get()->result));
            INFO("Play the voice of going to start recording face");
            // 开始录入人脸，请正对摄像头，且勿遮挡人脸，保持稳定
            this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_ADD_FACE;
            this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
            this->face_entry_time_ = time(nullptr);
            break;
          }
        case FaceEntrySrv::Request::CANCLE_ADD_FACE: {
            INFO(
              "Receive the return response of the request to cancel face entry. %d",
              static_cast<int>(ai_response.get()->result));
            // 播报引导用户取消录入语音
            this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_CANCLE_ADD_FACE;
            this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
            break;
          }
        case FaceEntrySrv::Request::CONFIRM_LAST_FACE: {
            INFO(
              "Receive the return response of the request to confirm last face. %d",
              static_cast<int>(ai_response.get()->result));
            INFO(
              "Receive the return response msg of the request to confirm last face.%s",
              (ai_response.get()->msg).c_str());
            // 播报引导用户确认录入语音，并考虑上传确认录入的人脸～～～～
            this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_CONFIRM_LAST_FACE;
            this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
            break;
          }
        case FaceEntrySrv::Request::UPDATE_FACE_ID: {
            INFO(
              "Receive the return response of the request to update face id %d",
              static_cast<int>(ai_response.get()->result));
            // 播报引导用户更新录入语音～～～～
            // this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_UPDATE_FACE_ID;
            // this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
            break;
          }
        case FaceEntrySrv::Request::DELETE_FACE: {
            INFO(
              "Receive the return response of the request to delete face.%d",
              static_cast<int>(ai_response.get()->result));
            // 人脸数据删除
            auto delete_result = obj.DeleteFace(request->username);
            INFO(
              "Return the result of deleting face.%d",
              static_cast<int>(delete_result));
            break;
          }
        case FaceEntrySrv::Request::GET_ALL_FACES: {
            INFO(
              "Receive the return response of the request to get all faces %d",
              static_cast<int>(ai_response.get()->result));
            // 播报引导用户获取人脸信息语音～～～～
            this->face_audiomsg_ptr_->play_id = AudioMsg::PID_FACE_ENTRY_GET_ALL_FACES;
            this->audio_pub_->publish(*(this->face_audiomsg_ptr_));
            break;
          }
        default: {
            break;
          }
      }
    } else {
      ERROR("Failed to call service /facemanager");
    }
    // ai模式置为inactive
    if (request->command != FaceEntrySrv::Request::ADD_FACE) {
      this->flag_ = 0;
      if (Is_Deactive()) {
        INFO("Successfully set AI model to Inactive");
      }
    }
    response->result = static_cast<int>(ai_response.get()->result);
    response->code = static_cast<int>(ai_response.get()->result);
    response->allfaces = ai_response.get()->msg;
  } catch (const std::exception & e) {
    WARN("face service is failed: <%s>", e.what());
  }
}
// inactive vision_manager lifecycle
bool face::Is_Active()
{
  // ai模式置为active
  if (!this->ai_get_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_get_state service not ready.");
  }
  INFO("Request to set AI Mode to active");
  auto get_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  auto cur_status = get_respone.wait_for(std::chrono::seconds(10));
  if (cur_status != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_change_state service not ready.");
    return false;
  }
  if (get_respone.get()->current_state.id == GetState::PRIMARY_STATE_INACTIVE) {
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = ChangeState::TRANSITION_ACTIVATE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(20)) == std::future_status::ready) {
      if (change_respone.get()->success) {
        INFO("Transition active successfully triggered.");
      } else {
        WARN("Failed to transform");
      }
    } else {
      WARN("Failed to call service /change_state");
    }
  }
  get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.get()->current_state.id != GetState::PRIMARY_STATE_ACTIVE) {
    ERROR("The AI algorithm is not in active state");
    return false;
  }
  return true;
}
// deactive vision_manager lifecycle
bool face::Is_Deactive()
{
  this->ai_face_recognition_request_->algo_enable.clear();
  this->ai_face_recognition_request_->algo_disable.clear();
  // ai模式置为inactive
  if (!this->ai_get_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_get_state service not ready.");
  }
  INFO("Request to set AI Mode to Inactive");
  auto get_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  auto cur_status = get_respone.wait_for(std::chrono::seconds(10));
  if (cur_status != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_change_state service not ready.");
    return false;
  }
  if (get_respone.get()->current_state.id == GetState::PRIMARY_STATE_ACTIVE) {
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = ChangeState::TRANSITION_DEACTIVATE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(20)) == std::future_status::ready) {
      if (change_respone.get()->success) {
        INFO("Transition inactive successfully triggered.");
      } else {
        WARN("Failed to transform");
      }
    } else {
      WARN("Failed to call service /change_state");
    }
  }
  get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.get()->current_state.id != GetState::PRIMARY_STATE_INACTIVE) {
    ERROR("The AI algorithm is not in active state");
    return false;
  }
  return true;
}
void face::Run()
{
  INFO("cyberdog_face node spin");
  this->executor_.add_node(node_ptr_);
  this->executor_.spin();
  rclcpp::shutdown();
}
}  // namespace interaction
}  // namespace cyberdog
