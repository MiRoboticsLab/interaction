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
#include <sstream>
#include <chrono>
#include "cyberdog_audio/cyberdog_audio.hpp"
#include "cyberdog_audio/instruction_def.hpp"
#include "cyberdog_audio/board_info.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "xpack/json.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_audio/machine_state.hpp"

namespace cyberdog
{
namespace interaction
{

ParametersInfo * ParametersInfo::parameters_info_ = nullptr;
std::mutex ParametersInfo::mutex_;

ParametersInfo * & ParametersInfo::GetInstance()
{
  if (parameters_info_ == nullptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (parameters_info_ == nullptr) {
      parameters_info_ = new (std::nothrow) ParametersInfo();
    }
  }
  return parameters_info_;
}

void ParametersInfo::DeleteInstance()
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (parameters_info_) {
    delete parameters_info_;
    parameters_info_ = nullptr;
  }
}

ParametersInfo::ParametersInfo(/* args */)
{
  // speech_handler_ptr_ = std::make_shared<SpeechHandler>(
  //   std::bind(&CyberdogAudio::SpeechPlayGoal, this, std::placeholders::_1));
}

ParametersInfo::~ParametersInfo()
{
}

}  // namespace interaction
}  // namespace cyberdog

using cyberdog::interaction::SelfCheckState;
using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

// #define  LCM_ADDR           "udpm://239.255.76.67:7667?ttl=1"
#define     VERSION_REQUEST_CODE    5100
#define     OTA_REQUEST_CODE        5101
#define     OTA_OVER_CODE           5102

cyberdog::interaction::CyberdogAudio::CyberdogAudio()
: Node(NODE_NAME),
  // lcm_(new lcm::LCM(LCM_ADDR)),
  // server(nullptr),
  recv_msg_cnt(0),
  check_state(SelfCheckState::SCSUnkown), is_wifi_connected(false),
  first_notified_net(false), failed_times(0),
  // cyberdog_sn(sn),
  triple_mac(""), triple_did(""), triple_key(""),
  is_authenticated(true), auth_uid(""),
  // is_play_(false),
  token_invalid_(false),
  switch_environment_(true), audio_environment_("product"),
  action_control_enable_(false),
  is_active_(false), battery_capicity_(100)
{
  code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<AudioErrorCode>>(
    cyberdog::system::ModuleCode::kAudioBoard);
  ready_sn_ptr =
    std::make_unique<cyberdog::interaction::ReadySnNode>();
  cyberdog_sn = ready_sn_ptr->WaitSn();
  INFO("sn:%s", cyberdog_sn.c_str());
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/interaction/audio.toml");
  toml::value config;
  if (common::CyberdogToml::ParseFile(path, config)) {
    INFO("toml settings file:%s ok", path.c_str());
    toml::value tv_settings;
    if (common::CyberdogToml::Get(config, "settings", tv_settings)) {
      toml::value tv_env;
      if (common::CyberdogToml::Get(tv_settings, "environment", tv_env)) {
        audio_environment_ = tv_env.as_string();
        INFO("toml get environment:%s", audio_environment_.c_str());
      }
      // toml::value tv_ace;
      // if (common::CyberdogToml::Get(tv_settings, "action_control", tv_ace)) {
      //   action_control_enable_ = (tv_ace.as_string() == "on" ? true : false);
      //   INFO("toml get action_control switch:%s", (action_control_enable_ ? "true" : "false"));
      // }
    }
  }
  voice_control_ptr_ = std::make_shared<VoiceControl>(action_control_enable_);
  path = voice_control_ptr_->CONFIG_DIR + "/" + voice_control_ptr_->CONFIG_FILE;
  Document json_document(kObjectType);
  auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
  if (result) {
    rapidjson::Value action_control_val(rapidjson::kObjectType);
    result = CyberdogJson::Get(json_document, "action_control", action_control_val);
    if (result) {
      action_control_enable_ = action_control_val.GetBool();
      INFO(
        "read file:%s, control enable:%s!", path.c_str(),
        (action_control_enable_ ? "on" : "off"));
      voice_control_ptr_->SetAcitionControl(action_control_enable_);
    }
  }
  aiconnect_state_pub_ =
    this->create_publisher<std_msgs::msg::Bool>(
    "ai_connect_state",
    rclcpp::SystemDefaultsQoS()
    );
  audio_board_state_pub_ =
    this->create_publisher<std_msgs::msg::UInt8>(
    "audio_board_state",
    rclcpp::SystemDefaultsQoS()
    );
  dlg_info_pub_ =
    this->create_publisher<std_msgs::msg::String>(
    "voice_dlg_info",
    rclcpp::SystemDefaultsQoS()
    );
  account_manager_ptr_ = std::make_unique<cyberdog::common::CyberdogAccountManager>();
  vp_database_ptr_ = std::make_unique<cyberdog::interaction::VoiceprintDatabase>();
  // speech_handler_ptr_ = std::make_shared<SpeechHandler>(
  //   std::bind(&CyberdogAudio::SpeechPlayGoal, this, std::placeholders::_1));
  audio_play_ptr_ = std::make_shared<AudioPlay>(
    std::bind(&CyberdogAudio::SpeechPlayGoal, this, std::placeholders::_1));
  audio_play_ptr_->callFunc(std::bind(&CyberdogAudio::GetPlayStatus, this));
  sdcard_playid_query_srv_ =
    this->create_service<protocol::srv::SdcardPlayIdQuery>(
    "sdcard_playid_query",
    std::bind(
      &CyberdogAudio::SdcardPlayidQuery, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, speech_callback_group_);
  audio_fds_ptr_ = std::make_unique<cyberdog::interaction::AudioFds>();
  audio_fds_ptr_->get_audio_play_ptr(audio_play_ptr_);
  RegisterAudioCyberdogTopicHandler();
  RegisterCyberdogAudioServiceReturnHandler();
  audio_state.RegisterNotice(
    std::bind(
      &cyberdog::interaction::CyberdogAudio::RegisterNotify, this,
      std::placeholders::_1));
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("audio");
  machine_state_ptr_ =
    std::make_unique<cyberdog::interaction::MachineState>("audio");
  machine_state_ptr_->Init();
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_SetUp,
    std::bind(&CyberdogAudio::OnSetUp, this));
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_SelfCheck,
    std::bind(&CyberdogAudio::OnSelfCheck, this));
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_Active,
    std::bind(&CyberdogAudio::OnActive, this));
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_DeActive,
    std::bind(&CyberdogAudio::OnDeactive, this));
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_TearDown,
    std::bind(&CyberdogAudio::OnTearDown, this));
  machine_state_ptr_->RegisterCallback(
    cyberdog::machine::MachineState::MS_OTA,
    std::bind(&CyberdogAudio::OnOta, this));
  if (!machine_state_ptr_->Start()) {
    INFO("machine state init failed!");
  } else {
    INFO("machine state init success.");
  }
  SetControlState(action_control_enable_);
  // auto srv_func_ = [this]() {
  //     server = std::make_shared<LcmServer>(
  //       ATOC_SERVICE,
  //       std::bind(
  //         &CyberdogAudio::ServerCallback, this, std::placeholders::_1,
  //         std::placeholders::_2));
  //     server->Spin();
  //   };
  // server_thread = std::thread(srv_func_);
  // auto msg_func_ = [this]() {
  //     if (!lcm_->good()) {
  //       ERROR("lcm is not good!");
  //       return;
  //     }
  //     lcm_->subscribe(
  //       ATOC_TOPIC, &cyberdog::interaction::CyberdogAudio::LcmHandler,
  //       this);
  //     while (0 == lcm_->handle()) {}
  //   };
  // message_thread = std::thread(msg_func_);
  // CreateLcm();
}

cyberdog::interaction::CyberdogAudio::~CyberdogAudio()
{
  // {
  //   std::unique_lock<std::mutex> lck(play_mtx_);
  //   is_play_ = false;
  //   play_cv_.notify_all();
  // }
  audio_play_ptr_->StopPlay();
}

void cyberdog::interaction::CyberdogAudio::CreateLcm()
{
  INFO("Create lcm!");
  {
    std::lock_guard<std::mutex> lck(lcm_ctoa_topic_mtx_);
    if (lcm_ctoa_topic_ == nullptr) {
      lcm_ctoa_topic_ = std::make_unique<LcmCtoaTopic>(
        std::bind(
          &CyberdogAudio::LcmPublish2, this, std::placeholders::_1));
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_atoc_topic_mtx_);
    if (lcm_atoc_topic_ == nullptr) {
      lcm_atoc_topic_ = std::make_unique<LcmAtocTopic>(
        std::bind(
          &CyberdogAudio::LcmHandler, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_ctoa_service_mtx_);
    if (lcm_ctoa_service_ == nullptr) {
      lcm_ctoa_service_ = std::make_unique<LcmCtoaService>(
        std::bind(
          &CyberdogAudio::ClientRequest2, this, std::placeholders::_1,
          std::placeholders::_2));
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_atoc_service_mtx_);
    if (lcm_atoc_service_ == nullptr) {
      lcm_atoc_service_ = std::make_unique<LcmAtocService>(
        std::bind(
          &CyberdogAudio::ServerCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    }
  }
}

void cyberdog::interaction::CyberdogAudio::DestroyLcm()
{
  INFO("Destroy lcm!");
  {
    std::lock_guard<std::mutex> lck(lcm_ctoa_topic_mtx_);
    if (lcm_ctoa_topic_ != nullptr) {
      lcm_ctoa_topic_.reset(nullptr);
      lcm_ctoa_topic_ = nullptr;
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_atoc_topic_mtx_);
    if (lcm_atoc_topic_ != nullptr) {
      lcm_atoc_topic_.reset(nullptr);
      lcm_atoc_topic_ = nullptr;
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_ctoa_service_mtx_);
    if (lcm_ctoa_service_ != nullptr) {
      lcm_ctoa_service_.reset(nullptr);
      lcm_ctoa_service_ = nullptr;
    }
  }
  {
    std::lock_guard<std::mutex> lck(lcm_atoc_service_mtx_);
    if (lcm_atoc_service_ != nullptr) {
      lcm_atoc_service_.reset(nullptr);
      lcm_atoc_service_ = nullptr;
    }
  }
}

int32_t cyberdog::interaction::CyberdogAudio::OnSetUp()
{
  INFO("Audio execute set up machine state method.");
  CreateLcm();
  OnActive();
  heart_beats_ptr_->HeartBeatRun();
  auto heart_func_ = [this]() {
      while (rclcpp::ok()) {
        try {
          // 开机自检心跳
          if (SelfCheck()) {
            if (switch_environment_) {
              if (GetEnvironment()) {
                switch_environment_ = false;
              } else if (SetEnvironment()) {
                switch_environment_ = false;
              }
            }
          }
          if (!first_notified_net && (check_state != SelfCheckState::SCSUnkown)) {
            // 开机广播一次网络连接
            NetStatusNotify();
            first_notified_net = true;
            protocol::msg::AudioPlay msg;
            msg.module_name = "self";
            msg.play_id = protocol::msg::AudioPlay::PID_SELF_CHECK_START;
            SpeechCallback(std::make_shared<protocol::msg::AudioPlay>(msg));
          }
        } catch (const std::exception & e) {
          INFO("heart func exception:%s.", e.what());
        } catch (...) {
          INFO("heart func unkown exception.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    };
  heart_thread = std::thread(heart_func_);
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnSelfCheck()
{
  // LcmPublish(speech_handler_ptr_->Play("语音模块自检完成,状态就绪!"));
  // {
  //   std::shared_ptr<protocol::srv::AudioNickName::Request> req =
  //     std::make_shared<protocol::srv::AudioNickName::Request>();
  //   req->nick_name = "铁蛋";
  //   req->wake_name = "铁蛋";
  //   std::shared_ptr<protocol::srv::AudioNickName::Response> res =
  //     std::make_shared<protocol::srv::AudioNickName::Response>();
  //   SetNickNameCallback(req, res);
  // }
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnActive()
{
  if (!is_active_) {
    speech_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    volumn_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    bc_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    power_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    train_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    speech_sub_ =
      this->create_subscription<protocol::msg::AudioPlay>(
      "speech_play", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::SpeechCallback, this, std::placeholders::_1));
    speech_extend_sub_ =
      this->create_subscription<protocol::msg::AudioPlayExtend>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::SpeechExtendCallback, this, std::placeholders::_1));
    wifi_sub_ =
      this->create_subscription<protocol::msg::WifiStatus>(
      "wifi_status", 1,
      std::bind(&CyberdogAudio::WifiCallback, this, std::placeholders::_1));
    wake_word_sub_ =
      this->create_subscription<std_msgs::msg::String>(
      "wake_word", 1,
      std::bind(&CyberdogAudio::WakeWordCallback, this, std::placeholders::_1));
    dog_info_sub_ =
      this->create_subscription<std_msgs::msg::String>(
      "dog_info", 1,
      std::bind(&CyberdogAudio::DogInfoCallback, this, std::placeholders::_1));
    ota_request_sub_ =
      this->create_subscription<std_msgs::msg::String>(
      "ota_audio_command_request", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::OtaRequestCallback, this, std::placeholders::_1));
    volume_set_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
      "volume_set", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::VolumeSetCallback, this, std::placeholders::_1));
    volume_get_sub_ =
      this->create_subscription<std_msgs::msg::UInt8>(
      "volume_get", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::VolumeGetCallback, this, std::placeholders::_1));
    restore_settings_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
      "audio_restore_settings", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::RestoreSettingsCallback, this, std::placeholders::_1));
    continue_dialog_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "continue_dialog", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::ContinueDialog, this, std::placeholders::_1));
    nlp_control_sub_ = this->create_subscription<std_msgs::msg::String>(
      "nlp_get", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::NlpControl, this, std::placeholders::_1));
    audio_set_status_srv_ =
      this->create_service<protocol::srv::AudioExecute>(
      "set_audio_state",
      std::bind(&CyberdogAudio::SetAudioState, this, std::placeholders::_1, std::placeholders::_2));
    audio_get_status_srv_ =
      this->create_service<protocol::srv::AudioExecute>(
      "get_audio_state",
      std::bind(&CyberdogAudio::GetAudioState, this, std::placeholders::_1, std::placeholders::_2));
    audio_auth_did_srv_ =
      this->create_service<protocol::srv::AudioAuthId>(
      "get_authenticate_didsn",
      std::bind(&CyberdogAudio::GetAuthDid, this, std::placeholders::_1, std::placeholders::_2));
    audio_auth_token_pub_ =
      this->create_publisher<protocol::msg::AuthToken>(
      "authenticate_token_enter",
      rclcpp::SystemDefaultsQoS()
      );
    audio_auth_token_srv_ =
      this->create_service<protocol::srv::AudioAuthToken>(
      "set_authenticate_token",
      std::bind(&CyberdogAudio::SetAuthToken, this, std::placeholders::_1, std::placeholders::_2));
    audio_voiceprint_train_srv_ =
      this->create_service<protocol::srv::AudioVoiceprintTrain>(
      "voiceprint_train",
      std::bind(
        &CyberdogAudio::VoiceprintCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_voiceprints_data_set_srv_ =
      this->create_service<protocol::srv::AudioVoiceprintsSet>(
      "voiceprints_data_notify",
      std::bind(
        &CyberdogAudio::VoiceprintsSetCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_nick_name_switch_srv_ =
      this->create_service<std_srvs::srv::SetBool>(
      "nick_name_switch",
      std::bind(
        &CyberdogAudio::SetNickNameSwitchCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_nick_name_set_srv_ =
      this->create_service<protocol::srv::AudioNickName>(
      "set_nick_name",
      std::bind(
        &CyberdogAudio::SetNickNameCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_volume_set_srv_ =
      this->create_service<protocol::srv::AudioVolumeSet>(
      "audio_volume_set",
      std::bind(
        &CyberdogAudio::AudioVolumeSet, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, volumn_callback_group_);
    audio_volume_get_srv_ =
      this->create_service<protocol::srv::AudioVolumeGet>(
      "audio_volume_get",
      std::bind(
        &CyberdogAudio::AudioVolumeGet, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, volumn_callback_group_);
    audio_text_play_srv_ =
      this->create_service<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      std::bind(
        &CyberdogAudio::AudioTextPlay, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, speech_callback_group_);
    audio_active_state_srv_ =
      this->create_service<protocol::srv::Trigger>(
      "audio_active_state",
      std::bind(
        &CyberdogAudio::AudiActiveState, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_action_set_srv_ =
      this->create_service<std_srvs::srv::SetBool>(
      "audio_action_set",
      std::bind(
        &CyberdogAudio::AudioActionSetCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_action_get_srv_ =
      this->create_service<std_srvs::srv::Trigger>(
      "audio_action_get",
      std::bind(
        &CyberdogAudio::AudioActionGetCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_voiceprint_entry_srv_ =
      this->create_service<protocol::srv::AudioVoiceprintEntry>(
      "audio_voiceprint_entry",
      std::bind(
        &CyberdogAudio::AudioVoiceEntry, this, std::placeholders::_1,
        std::placeholders::_2));
    switch_environment_srv_ =
      this->create_service<protocol::srv::Trigger>(
      "set_work_environment",
      std::bind(
        &CyberdogAudio::SwitchEnvironmentService, this, std::placeholders::_1,
        std::placeholders::_2));
    stop_play_srv_ =
      this->create_service<std_srvs::srv::Empty>(
      "stop_play",
      std::bind(
        &CyberdogAudio::StopPlayService, this, std::placeholders::_1,
        std::placeholders::_2));
    audio_voiceprint_result_pub_ =
      this->create_publisher<protocol::msg::AudioVoiceprintResult>(
      "audio_voiceprint_result",
      rclcpp::SystemDefaultsQoS()
      );
    audio_voiceprints_data_get_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(
      "voiceprints_data_require",
      rclcpp::SystemDefaultsQoS()
      );
    ota_response_pub_ =
      this->create_publisher<std_msgs::msg::String>(
      "ota_audio_command_response",
      rclcpp::SystemDefaultsQoS()
      );
    audio_asr_pub_ =
      this->create_publisher<std_msgs::msg::String>(
      "asr_text",
      rclcpp::SystemDefaultsQoS()
      );
    uid_pub_ =
      this->create_publisher<std_msgs::msg::String>(
      "uid_set",
      rclcpp::SystemDefaultsQoS()
      );
    dog_info_update_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(
      "dog_info_update",
      rclcpp::SystemDefaultsQoS()
      );
    wake_up_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(
      "dog_wakeup",
      rclcpp::SystemDefaultsQoS()
      );
    train_plan_pub_ =
      this->create_publisher<protocol::msg::TrainPlan>(
      "train_plan_word",
      rclcpp::SystemDefaultsQoS()
      );
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = bc_callback_group_;
    bms_status_sub_ = this->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogAudio::BmsStatus, this, std::placeholders::_1),
      sub_options);
    power_client_ = this->create_client<std_srvs::srv::Trigger>(
      "poweroff",
      rmw_qos_profile_services_default,
      power_callback_group_
    );
    reboot_client_ = this->create_client<std_srvs::srv::Trigger>(
      "reboot",
      rmw_qos_profile_services_default,
      power_callback_group_
    );
    train_plan_client_ = this->create_client<protocol::srv::TrainPlan>(
      "query_train_plan",
      rmw_qos_profile_services_default,
      train_callback_group_
    );
    is_active_ = true;
  }
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnDeactive()
{
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnTearDown()
{
  if (audio_play_ptr_->GetPlayStatus() == 1) {
    std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
    l_d->cmd = PLAY_CANCEL;
    LcmPublish(l_d);
  }
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnLowPower()
{
  return 0;
}

int32_t cyberdog::interaction::CyberdogAudio::OnOta()
{
  if (voice_control_ptr_) {
    voice_control_ptr_->SetAcitionControl(false);
    SetControlState(false);
  }
  return 0;
}

void cyberdog::interaction::CyberdogAudio::SpeechPlayGoal(
  const std::shared_ptr<SpeechActionServer::GoalHandleSpeech> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  if (goal->is_online) {
    INFO(
      "action online module name:%s, speech text: %s",
      goal->module_name.c_str(), goal->text.c_str());
    // LcmPublish(speech_handler_ptr_->Play(goal->text));
    play_sound_info psi;
    psi.is_online = goal->is_online;
    psi.play_text = goal->text;
    LcmPublish(audio_play_ptr_->SoundPlay(psi));
  } else {
    SpeechCallback(std::make_shared<protocol::msg::AudioPlay>(goal->speech));
  }
}

void cyberdog::interaction::CyberdogAudio::SpeechCallback(
  const protocol::msg::AudioPlay::SharedPtr msg)
{
  INFO("module name:%s, speech play: %d", msg->module_name.c_str(), msg->play_id);
  play_sound_info psi;
  psi.play_id = msg->play_id;
  LcmPublish(audio_play_ptr_->SoundPlay(psi));
}

void cyberdog::interaction::CyberdogAudio::SpeechExtendCallback(
  const protocol::msg::AudioPlayExtend::SharedPtr msg)
{
  if (msg->is_online) {
    INFO(
      "topic online module name:%s, speech text: %s", msg->module_name.c_str(),
      msg->text.c_str());
    play_sound_info psi;
    psi.is_online = msg->is_online;
    psi.play_text = msg->text;
    LcmPublish(audio_play_ptr_->SoundPlay(psi));
  } else {
    SpeechCallback(std::make_shared<protocol::msg::AudioPlay>(msg->speech));
  }
}

void cyberdog::interaction::CyberdogAudio::WifiCallback(
  const protocol::msg::WifiStatus::SharedPtr msg)
{
  if (first_notified_net) {
    if (msg->is_connected != is_wifi_connected) {
      is_wifi_connected = msg->is_connected;
      // 网络连接边际广播
      NetStatusNotify();
      if (is_wifi_connected) {
        static bool is_update_once = false;
        if (!is_update_once) {
          is_update_once = true;
          audio_fds_ptr_->Update();
        }
      }
    }
  }
}

void cyberdog::interaction::CyberdogAudio::WakeWordCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  SetWake(msg->data);
}

void cyberdog::interaction::CyberdogAudio::DogInfoCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  personal_info.name = msg->data;
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y-%m-%d %X");
  personal_info.activate_date = ss.str();
  personal_info.weight = CONST_WEIGHT;
  DogInfoNotify();
}

void cyberdog::interaction::CyberdogAudio::OtaRequestCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  ota_data od;
  xpack::json::decode(msg->data, od);
  if (od.command == VERSION_REQUEST_CODE) {
    VerRequestNotify();
  } else if (od.command == OTA_REQUEST_CODE) {
    OtaRequestNotify();
  } else if (od.command == OTA_OVER_CODE) {
    OtaOverNotify();
  }
}
void cyberdog::interaction::CyberdogAudio::StopPlayService(
  const std_srvs::srv::Empty::Request::SharedPtr request,
  std_srvs::srv::Empty::Response::SharedPtr response)
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = PLAY_CANCEL;
  LcmPublish(l_d);
}
void cyberdog::interaction::CyberdogAudio::SetAudioState(
  const protocol::srv::AudioExecute::Request::SharedPtr request,
  protocol::srv::AudioExecute::Response::SharedPtr respose)
{
  bool result = SetStatus(request->status.state);
  if (result) {
    audio_state.SetAudioWorkState((AudioWorkState)(request->status.state));
    respose->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  } else {
    ERROR("set audio work state failed!");
    respose->code = code_ptr_->GetCode(AudioErrorCode::kAudioMicOnOffError);
  }
  respose->result = result;
}

void cyberdog::interaction::CyberdogAudio::GetAudioState(
  const protocol::srv::AudioExecute::Request::SharedPtr,
  protocol::srv::AudioExecute::Response::SharedPtr respose)
{
  GetStatus();
  respose->result = audio_state.GetAudioWorkState();
  if (respose->result) {
    respose->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  } else {
    respose->code = code_ptr_->GetCode(AudioErrorCode::kAudioMicStateError);
  }
}

void cyberdog::interaction::CyberdogAudio::GetAuthDid(
  const protocol::srv::AudioAuthId::Request::SharedPtr,
  protocol::srv::AudioAuthId::Response::SharedPtr response)
{
  response->did = triple_did;
  response->sn = cyberdog_sn;
}

void cyberdog::interaction::CyberdogAudio::SetAuthToken(
  const protocol::srv::AudioAuthToken::Request::SharedPtr request,
  protocol::srv::AudioAuthToken::Response::SharedPtr response)
{
  auth_uid = request->uid;
  auth_ti.uid = auth_uid;
  auth_ti.token_access = request->token_access;
  auth_ti.token_fresh = request->token_fresh;
  auth_ti.token_expirein = request->token_expirein;
  // auth_ti.token_deviceid = triple_did;
  auth_ti.token_deviceid = cyberdog_sn;
  INFO(
    "uid:%s,access:%s,fresh:%s,expirein:%ld,deviceid:%s", auth_ti.uid.c_str(),
    auth_ti.token_access.c_str(), auth_ti.token_fresh.c_str(),
    auth_ti.token_expirein, auth_ti.token_deviceid.c_str());
  protocol::msg::AuthToken at_enter;
  at_enter.uid = auth_ti.uid;
  at_enter.token_access = auth_ti.token_access;
  at_enter.token_fresh = auth_ti.token_fresh;
  at_enter.token_expirein = auth_ti.token_expirein;
  audio_auth_token_pub_->publish(at_enter);
  if (auth_ti.token_deviceid == cyberdog_sn) {
    // response->result = true;
    // AccountTokenNotify();
    response->result = SetToken();
    if (!response->result) {
      ERROR("set token failed!");
      response->code = code_ptr_->GetCode(AudioErrorCode::kAudioSetTokenError);
    } else {
      std_msgs::msg::String msg;
      msg.data = auth_uid;
      uid_pub_->publish(msg);
      response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
    }
  } else {
    ERROR(
      "device id:%s, token did:%s not equal.", triple_did.c_str(),
      auth_ti.token_deviceid.c_str());
    response->result = false;
    response->code = code_ptr_->GetCode(AudioErrorCode::kAudioSetTokenError);
  }
}

void cyberdog::interaction::CyberdogAudio::VoiceprintCallback(
  const protocol::srv::AudioVoiceprintTrain::Request::SharedPtr request,
  protocol::srv::AudioVoiceprintTrain::Response::SharedPtr)
{
  if (request->train_id == protocol::srv::AudioVoiceprintTrain::Request::TID_START) {
    INFO(
      "voiceprint train start,(id:%s,name:%s)", request->voice_print.id.c_str(),
      request->voice_print.name.c_str());
    VoiceprintRegisterNotify(request->voice_print.name);
    voice_print_training_info_.name = request->voice_print.name;
    voice_print_training_info_.id = request->voice_print.id;
    INFO("account modify information audio state in processing");
    account_manager_ptr_->ModifyUserInformation(request->voice_print.name, 2, 0);
  } else if (request->train_id == protocol::srv::AudioVoiceprintTrain::Request::TID_CANCEL) {
    INFO(
      "voiceprint train cancel,(id:%s,name:%s)", request->voice_print.id.c_str(),
      request->voice_print.name.c_str());
    VoiceprintCancelNotify();
    INFO("account modify information audio state in not processing");
    account_manager_ptr_->ModifyUserInformation(request->voice_print.name, 0, 0);
  } else {
    ERROR(
      "voiceprint train error, unkown train id:%d, voiceprint(%s:%s)",
      request->train_id, request->voice_print.id.c_str(),
      request->voice_print.name.c_str());
  }
}

void cyberdog::interaction::CyberdogAudio::VoiceprintsSetCallback(
  const protocol::srv::AudioVoiceprintsSet::Request::SharedPtr request,
  protocol::srv::AudioVoiceprintsSet::Response::SharedPtr response)
{
  (void) request;
  response->result = true;
}

void cyberdog::interaction::CyberdogAudio::SetNickNameSwitchCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  INFO("nick name switch %s", request->data == true ? "on" : "off");
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
  Document json_document(kObjectType);
  Document::AllocatorType & allocator = json_document.GetAllocator();
  auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
  rapidjson::Value dog_val(rapidjson::kObjectType);
  if (result) {
    result = CyberdogJson::Get(json_document, "dog_info", dog_val);
    if (result) {
      if (dog_val.HasMember("enable")) {
        dog_val["enable"].SetBool(request->data);
      } else {
        dog_val.AddMember("enable", request->data, allocator);
      }
    } else {
      dog_val.AddMember("enable", request->data, allocator);
    }
  } else {
    INFO("no file:%s, create it!", path.c_str());
    dog_val.AddMember("enable", request->data, allocator);
  }
  std::string nick_name("铁蛋");
  std::string wake_name("铁蛋铁蛋");
  if (request->data == true) {
    if (dog_val.HasMember("nick_name")) {
      nick_name = dog_val["nick_name"].GetString();
    }
    if (dog_val.HasMember("wake_name")) {
      wake_name = dog_val["wake_name"].GetString();
    }
  }
  personal_info.name = nick_name;
  if (!dog_val.HasMember("nick_name")) {
    dog_val.AddMember("nick_name", nick_name, allocator);
  }
  if (!dog_val.HasMember("wake_name")) {
    dog_val.AddMember("wake_name", wake_name, allocator);
  }
  if (!dog_val.HasMember("activate_date")) {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d %X");
    personal_info.activate_date = ss.str();
    dog_val.AddMember("activate_date", personal_info.activate_date, allocator);
  } else {
    personal_info.activate_date = dog_val["activate_date"].GetString();
  }
  if (!dog_val.HasMember("weight")) {
    personal_info.weight = CONST_WEIGHT;
    dog_val.AddMember("weight", personal_info.weight, allocator);
  } else {
    personal_info.weight = dog_val["weight"].GetFloat();
  }
  INFO(
    "name switch after, name:%s(%s), activate date:%s, weight:%f",
    personal_info.name.c_str(), wake_name.c_str(),
    personal_info.activate_date.c_str(), personal_info.weight);
  SetWake(wake_name);
  CyberdogJson::Add(json_document, "dog_info", dog_val);
  CyberdogJson::WriteJsonToFile(path, json_document);
  response->success = true;
  std_msgs::msg::Bool msg;
  msg.data = true;
  dog_info_update_pub_->publish(msg);
  DogInfoNotify();
}

void cyberdog::interaction::CyberdogAudio::SetNickNameCallback(
  const protocol::srv::AudioNickName::Request::SharedPtr request,
  protocol::srv::AudioNickName::Response::SharedPtr response)
{
  INFO("set nick name %s(%s)", request->nick_name.c_str(), request->wake_name.c_str());
  SetWake(request->wake_name);
  personal_info.name = request->nick_name;
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y-%m-%d %X");
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
  Document json_document(kObjectType);
  auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
  rapidjson::Value dog_val(rapidjson::kObjectType);
  if (result) {
    result = CyberdogJson::Get(json_document, "dog_info", dog_val);
    if (result) {
      Document::AllocatorType & allocator = json_document.GetAllocator();
      if (dog_val.HasMember("nick_name")) {
        dog_val["nick_name"].SetString(request->nick_name, allocator);
      } else {
        dog_val.AddMember("nick_name", request->nick_name, allocator);
      }
      if (dog_val.HasMember("wake_name")) {
        dog_val["wake_name"].SetString(request->wake_name, allocator);
      } else {
        dog_val.AddMember("wake_name", request->wake_name, allocator);
      }
      if (dog_val.HasMember("activate_date")) {
        personal_info.activate_date = dog_val["activate_date"].GetString();
      } else {
        personal_info.activate_date = ss.str();
        dog_val.AddMember("activate_date", personal_info.activate_date, allocator);
      }
      if (dog_val.HasMember("weight")) {
        personal_info.weight = dog_val["weight"].GetFloat();
      } else {
        personal_info.weight = CONST_WEIGHT;
        dog_val.AddMember("weight", personal_info.weight, allocator);
      }
    } else {
      Document::AllocatorType & allocator = json_document.GetAllocator();
      personal_info.activate_date = ss.str();
      personal_info.weight = CONST_WEIGHT;
      dog_val.AddMember("nick_name", request->nick_name, allocator);
      dog_val.AddMember("wake_name", request->wake_name, allocator);
      dog_val.AddMember("activate_date", personal_info.activate_date, allocator);
      dog_val.AddMember("weight", personal_info.weight, allocator);
    }
  } else {
    INFO("no file:%s, create it!", path.c_str());
    personal_info.activate_date = ss.str();
    personal_info.weight = CONST_WEIGHT;
    Document::AllocatorType & allocator = json_document.GetAllocator();
    dog_val.AddMember("nick_name", request->nick_name, allocator);
    dog_val.AddMember("wake_name", request->wake_name, allocator);
    dog_val.AddMember("activate_date", personal_info.activate_date, allocator);
    dog_val.AddMember("weight", personal_info.weight, allocator);
  }
  DogInfoNotify();
  INFO("set nick name success!");
  CyberdogJson::Add(json_document, "dog_info", dog_val);
  CyberdogJson::WriteJsonToFile(path, json_document);
  response->success = true;
  response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  std_msgs::msg::Bool msg;
  msg.data = true;
  dog_info_update_pub_->publish(msg);
}

void cyberdog::interaction::CyberdogAudio::AudioVolumeSet(
  const protocol::srv::AudioVolumeSet::Request::SharedPtr request,
  protocol::srv::AudioVolumeSet::Response::SharedPtr response)
{
  response->success = SetVolume(request->volume);
  if (response->success) {
    response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  } else {
    response->code = code_ptr_->GetCode(AudioErrorCode::kAudioSetVoiceError);
  }
}

void cyberdog::interaction::CyberdogAudio::AudioVolumeGet(
  const protocol::srv::AudioVolumeGet::Request::SharedPtr,
  protocol::srv::AudioVolumeGet::Response::SharedPtr response)
{
  response->volume = GetVolume();
}

void cyberdog::interaction::CyberdogAudio::AudioTextPlay(
  const protocol::srv::AudioTextPlay::Request::SharedPtr request,
  protocol::srv::AudioTextPlay::Response::SharedPtr response)
{
  // {
  //   std::unique_lock<std::mutex> lck(play_mtx_);
  //   is_play_ = true;
  // }
  audio_play_ptr_->StartPlay();
  if (request->is_online) {
    INFO(
      "service online module name:%s, speech text: %s",
      request->module_name.c_str(), request->text.c_str());
    // LcmPublish(speech_handler_ptr_->Play(request->text));
    play_sound_info psi;
    psi.is_online = request->is_online;
    psi.play_text = request->text;
    LcmPublish(audio_play_ptr_->SoundPlay(psi));
  } else {
    SpeechCallback(std::make_shared<protocol::msg::AudioPlay>(request->speech));
  }
  uint8_t status = 0;
  {
    // std::unique_lock<std::mutex> lck(play_mtx_);
    // if (play_cv_.wait_for(
    //     lck, std::chrono::seconds(10), [this]() -> bool {
    //       return (is_play_ == false) || (!rclcpp::ok());
    //     }) == false)
    if (!audio_play_ptr_->WaitPlay()) {
      status = 1;
    }
  }
  response->status = status;
}

void cyberdog::interaction::CyberdogAudio::SdcardPlayidQuery(
  const protocol::srv::SdcardPlayIdQuery::Request::SharedPtr request,
  protocol::srv::SdcardPlayIdQuery::Response::SharedPtr response)
{
  response->exist = false;
  if (audio_play_ptr_->IsSdcardHaveId(request->play_id)) {
    response->exist = true;
  }
}

void cyberdog::interaction::CyberdogAudio::AudiActiveState(
  const protocol::srv::Trigger::Request::SharedPtr request,
  protocol::srv::Trigger::Response::SharedPtr response)
{
  if (request->data != "" && auth_uid != request->data) {
    response->success = false;
  } else {
    response->success = (token_invalid_ == true ? false : audio_state.GetActivateState());
  }
  INFO(
    "uid:%s, activate:%s", (request->data == "" ? "empty" : request->data.c_str()),
    (response->success ? "true" : "false"));
  response->message = "ok";
}

void cyberdog::interaction::CyberdogAudio::AudioActionSetCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  (void) request;
  auto path = voice_control_ptr_->CONFIG_DIR + "/" + voice_control_ptr_->CONFIG_FILE;
  if (access(voice_control_ptr_->CONFIG_DIR.c_str(), F_OK) != 0) {
    std::string cmd = "mkdir -p " + voice_control_ptr_->CONFIG_DIR;
    std::system(cmd.c_str());
    cmd = "chmod 777 " + voice_control_ptr_->CONFIG_DIR;
    std::system(cmd.c_str());
  }
  Document json_document(kObjectType);
  // Document::AllocatorType & allocator = json_document.GetAllocator();
  auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
  rapidjson::Value action_control_val(rapidjson::kObjectType);
  if (result) {
    // action_control_val.SetBool(request->data);
  } else {
    INFO("no file:%s, create it!", path.c_str());
  }
  action_control_val.SetBool(request->data);
  CyberdogJson::Add(json_document, "action_control", action_control_val);
  CyberdogJson::WriteJsonToFile(path, json_document);
  voice_control_ptr_->SetAcitionControl(request->data);
  bool state_ = request->data;
  SetControlState(state_);
  response->success = true;
  response->message = "success";
}

void cyberdog::interaction::CyberdogAudio::AudioActionGetCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void) request;
  response->success = voice_control_ptr_->GetActionControl();
  response->message = "success";
}

void cyberdog::interaction::CyberdogAudio::AudioVoiceEntry(
  const protocol::srv::AudioVoiceprintEntry::Request::SharedPtr request,
  protocol::srv::AudioVoiceprintEntry::Response::SharedPtr response)
{
  response->success = false;
  if (request->command == protocol::srv::AudioVoiceprintEntry::Request::AVE_DELETE_VOICEPRINT) {
    INFO("delete voice name:%s", request->username.c_str());
    response->success = account_manager_ptr_->DeleteVoice(request->username);
  }
}

void cyberdog::interaction::CyberdogAudio::SwitchEnvironmentService(
  const protocol::srv::Trigger::Request::SharedPtr request,
  protocol::srv::Trigger::Response::SharedPtr response)
{
  INFO("switch ai environment:");
  std::string tmp_env = request->data;
  if (request->data == "test" || request->data == "pro") {
    INFO("environment:%s", tmp_env.c_str());
    response->success = true;
    response->message = "environment ok not switch.";
    return;
    tmp_env = (request->data == "test") ? "preview" : "product";
    if (audio_environment_ != tmp_env) {
      audio_environment_ = tmp_env;
      bool result = SetEnvironment();
      if (result) {
        response->success = true;
      } else {
        response->success = false;
      }
    } else {
      response->success = true;
    }
  } else if (tmp_env == "product" || tmp_env == "preview" || tmp_env == "preview4test") {
    if (audio_environment_ != request->data) {
      audio_environment_ = request->data;
      bool result = SetEnvironment();
      if (result) {
        response->success = true;
      } else {
        response->success = false;
      }
    } else {
      response->success = true;
    }
  } else {
    response->success = false;
    response->message = "environment does not exist!";
  }
}

void cyberdog::interaction::CyberdogAudio::PowerTimerCallback()
{
  INFO("power instruction:shutdown");
  if (!power_client_->wait_for_service(std::chrono::seconds(3))) {
    INFO(
      "call power client server not avaiable"
    );
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = power_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call power client request services.");
  } else {
    INFO(
      "Failed to call power client request  services.");
  }
}

void cyberdog::interaction::CyberdogAudio::RebootTimerCallback()
{
  INFO("power instruction:reboot");
  if (!reboot_client_->wait_for_service(std::chrono::seconds(3))) {
    INFO(
      "call reboot client server not avaiable"
    );
    return;
  }
  std::chrono::seconds timeout(3);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = reboot_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(timeout);
  if (status == std::future_status::ready) {
    INFO(
      "success to call reboot client request services.");
  } else {
    INFO(
      "Failed to call reboot client request  services.");
  }
}

void cyberdog::interaction::CyberdogAudio::VolumeSetCallback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  SetVolume(msg->data);
}
void cyberdog::interaction::CyberdogAudio::VolumeGetCallback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  (void) msg;
  GetVolume();
}

void cyberdog::interaction::CyberdogAudio::RestoreSettingsCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  (void) msg;
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = RESTORE_SETTINGS;
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::BmsStatus(
  const protocol::msg::BmsStatus::SharedPtr msg)
{
  battery_capicity_ = msg->batt_soc;
}
void cyberdog::interaction::CyberdogAudio::NlpControl(
  const std_msgs::msg::String::SharedPtr msg)
{
  nlp_control cd;
  cd.text = msg->data;
  INFO("nlpsontrol: %s", (msg->data).c_str());
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = NLP_CONTROL;
  l_d->data = xpack::json::encode(cd);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::ContinueDialog(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  continue_dialog cd;
  cd.set = msg->data;
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = CONTINUE_DIALOG;
  l_d->data = xpack::json::encode(cd);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::LcmHandler(
  const lcm::ReceiveBuffer * rbuf, const std::string & chan, const audio_lcm::lcm_data * msg)
{
  (void)rbuf;
  if (audio_cyberdog_topic_cmd_map.find(msg->cmd) != audio_cyberdog_topic_cmd_map.end()) {
    audio_cyberdog_topic_cmd_map[msg->cmd](msg->data);
  } else if (msg->cmd == TOKEN_STATE) {
    // 账号token失效通知
    INFO(
      "subcribe:%s(%d): cmd[%s], data[%s]",
      chan.c_str(), recv_msg_cnt++,
      msg->cmd.c_str(), msg->data.c_str());
  } else if (msg->cmd == AI_INSTRUCTION) {
    // 小爱服务器下发指令
  } else if (msg->cmd == ASR_NOTIFY) {
    // ASR通知
    asr_notify an;
    xpack::json::decode(msg->data, an);
    if (an.final) {
      std_msgs::msg::String msg;
      msg.data = an.text;
      audio_asr_pub_->publish(msg);
    }
  } else if (msg->cmd == DLG_INFO) {
    // 对话关联的声纹信息
    dlg_info di;
    xpack::json::decode(msg->data, di);
    INFO(
      "dialog info {dialogid:%s,voiceid:%s,nickname:%s}",
      di.dialogid.c_str(), di.voiceid.c_str(),
      di.nickname.c_str());
    if (!di.nickname.empty()) {
      std_msgs::msg::String di_msg;
      di_msg.data = di.nickname;
      dlg_info_pub_->publish(di_msg);
    }
  } else if (msg->cmd == AI_STATE) {
    // 云端连接状态通知
    ai_state as;
    xpack::json::decode(msg->data, as);
    INFO_EXPRESSION(
      as.connected == token_invalid_,
      "connection state:%s", (as.connected == true ? "true" : "false"));
    // if (token_invalid_ && as.connected) {
    //   auto msg = std::make_shared<protocol::msg::AudioPlay>();
    //   msg->module_name = "self";
    //   msg->play_id = protocol::msg::AudioPlay::PID_AI_ENABLE_SUCCESS;
    //   SpeechCallback(msg);
    // }
    if (true) {
      INFO("publish connection state:%s", as.connected ? "true" : "false");
      std_msgs::msg::Bool msg;
      msg.data = true;
      aiconnect_state_pub_->publish(msg);
    }
    token_invalid_ = !as.connected;
  } else if (msg->cmd == AI_ERROR) {
    // 云端返回错误通知
    ai_error ae;
    xpack::json::decode(msg->data, ae);
    WARN(
      "ai server report (code-%d, message:%s, eventid:%s",
      ae.code, ae.message.c_str(), ae.eventid.c_str());
    if (ae.code == 401) {
      token_invalid_ = true;
      auto msg = std::make_shared<protocol::msg::AudioPlay>();
      msg->module_name = "self";
      msg->play_id = protocol::msg::AudioPlay::PID_AI_PLEASE_ENABLE;
      SpeechCallback(msg);
    }
  } else if (msg->cmd == SETTINGS_RESTORE_NOTIFY) {
    // 恢复出厂设置完成通知
    settings_restore_notify srn;
    xpack::json::decode(msg->data, srn);
  } else if (msg->cmd == AUDIO_BOARD_STATE) {
    // 音箱当前的状态
    audio_board_state abs;
    xpack::json::decode(msg->data, abs);
    INFO("audio board state:%d", abs.state);
    std_msgs::msg::UInt8 abs_msg;
    abs_msg.data = abs.state;
    audio_board_state_pub_->publish(abs_msg);
  } else if (msg->cmd == POWER_INSTRUCTION) {
    // 电源指令
    power_instruction pi;
    xpack::json::decode(msg->data, pi);
    INFO(
      "power instruction:(operation:%s, delay:%d, confirmation:%s)",
      pi.operation.c_str(), pi.delay, (pi.confirmation ? "true" : "false"));
    if (pi.operation == "SHUTDOWN") {
      if (voice_control_ptr_->GetActionControl()) {
        if (pi.delay == 0) {
          PowerTimerCallback();
        } else {
          power_timer_ = create_wall_timer(
            std::chrono::seconds(pi.delay),
            std::bind(&CyberdogAudio::PowerTimerCallback, this), timer_callback_group_);
        }
      }
    } else if (pi.operation == "REBOOT") {
      if (voice_control_ptr_->GetActionControl()) {
        if (pi.delay == 0) {
          RebootTimerCallback();
        } else {
          power_timer_ = create_wall_timer(
            std::chrono::seconds(pi.delay),
            std::bind(&CyberdogAudio::RebootTimerCallback, this), timer_callback_group_);
        }
      }
    }
  } else if (msg->cmd == PERSONALIZE_INSTRUCTION) {
    INFO(
      "train cmd[%s], data[%s]", msg->cmd.c_str(), msg->data.c_str());
    personalize_instruction pi;
    xpack::json::decode(msg->data, pi);
    for (auto & train_cmd : pi.cmds) {
      personalize_instruction_keyval pik;
      xpack::json::decode(train_cmd, pik);
      INFO("train type:%s,value:%s", pik.exectype.c_str(), pik.execvalue.c_str());
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<protocol::srv::TrainPlan::Request>();
      req->type = pik.exectype;
      req->value = pik.execvalue;
      auto future_result = train_plan_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call train plan query services.");
        // Todo 添加调用动作的函数
        auto motion_ = future_result.get()->training;
        voice_control_ptr_->Personal(motion_.type, motion_.value);
        train_plan_pub_->publish(future_result.get()->training);
      } else {
        INFO(
          "Failed to call train plan query services.");
      }
    }
  } else {
    WARN(
      "subcribe unimplement cmd:%s, data:%s",
      (msg->cmd.empty() ? "empty" : msg->cmd.c_str()),
      (msg->data.empty() ? "empty" : msg->data.c_str()));
  }
}

// void cyberdog::interaction::CyberdogAudio::LcmPublish(
//   const std::shared_ptr<audio_lcm::lcm_data> & data)
// {
//   try {
//     if (check_state == SelfCheckState::SCSUnkown) {
//       ERROR(
//         "audio board in %s state.ignore publish message.",
//         audio_state.GetSelfCheckState().c_str());
//       return;
//     }
//     DEBUG(
//       "publish: cmd[%s], data[%s]",
//       data->cmd.c_str(), data->data.c_str());
//     lcm_->publish(CTOA_TOPIC, data.get());
//   } catch (const std::exception & e) {
//     std::cerr << CTOA_TOPIC << ":" << e.what() << '\n';
//   } catch (...) {
//     WARN("%s:unkown exception", CTOA_TOPIC);
//   }
// }

void cyberdog::interaction::CyberdogAudio::LcmPublish(
  const std::shared_ptr<audio_lcm::lcm_data> & data)
{
  std::lock_guard<std::mutex> lck(lcm_ctoa_topic_mtx_);
  if (lcm_ctoa_topic_ != nullptr) {
    lcm_ctoa_topic_->LcmPublish(data);
  }
}

int32_t cyberdog::interaction::CyberdogAudio::LcmPublish2(
  const std::shared_ptr<audio_lcm::lcm_data> & data)
{
  (void)data;
  if (check_state == SelfCheckState::SCSUnkown) {
    ERROR(
      "audio board in %s state.ignore publish message.",
      audio_state.GetSelfCheckState().c_str());
    return -1;
  }
  return 0;
}

// void cyberdog::interaction::CyberdogAudio::ServerCallback(
//   const audio_lcm::lcm_data & req,
//   audio_lcm::lcm_data & res)
// {
//   try {
//     DEBUG(
//       "server receive: cmd[%s], data[%s]", req.cmd.c_str(),
//       (req.data.empty() ? "" : req.data.c_str()));
//     if (req.cmd == GET_NETSTATUS) {
//       // 查询网络连接
//       get_net_status data;
//       data.result = is_wifi_connected;
//       res.cmd = req.cmd;
//       res.data = xpack::json::encode(data);
//     } else if (req.cmd == GET_TOKEN) {
//       // 获取小米账号token服务
//       token_info_state tis;
//       tis.uid = auth_ti.uid;
//       tis.access = auth_ti.token_access;
//       tis.fresh = auth_ti.token_fresh;
//       tis.expirein = auth_ti.token_expirein;
//       // tis.deviceid = auth_ti.token_deviceid;
//       tis.deviceid = cyberdog_sn;
//       tis.state = (is_authenticated == true) ? 0 : -1;
//       res.cmd = req.cmd;
//       res.data = xpack::json::encode(tis);
//     } else if (req.cmd == GET_PARAM) {
//       // 获取主控板的参数信息
//       cyberdog_param_info cpi;
//       cpi.sn = cyberdog_sn;
//       cpi.mac = triple_mac;
//       cpi.did = triple_did;
//       cpi.key = triple_key;
//       res.data = xpack::json::encode(cpi);
//     } else if (req.cmd == GET_DOG) {
//       // 机器狗的个狗信息获取
//       dog_info di;
//       di.name = personal_info.name;
//       di.activate_date = personal_info.activate_date;
//       di.weight = personal_info.weight;
//       res.data = xpack::json::encode(di);
//     }
//   } catch (const std::exception & e) {
//     std::cerr << ATOC_SERVICE << ":" << e.what() << '\n';
//   } catch (...) {
//     WARN("%s:unkown exception", ATOC_SERVICE);
//   }
// }

void cyberdog::interaction::CyberdogAudio::ServerCallback(
  const audio_lcm::lcm_data & req,
  audio_lcm::lcm_data & res)
{
  if (req.cmd == GET_NETSTATUS) {
    // 查询网络连接
    get_net_status data;
    data.result = is_wifi_connected;
    res.cmd = req.cmd;
    res.data = xpack::json::encode(data);
  } else if (req.cmd == GET_TOKEN) {
    // 获取小米账号token服务
    token_info_state tis;
    tis.uid = auth_ti.uid;
    tis.access = auth_ti.token_access;
    tis.fresh = auth_ti.token_fresh;
    tis.expirein = auth_ti.token_expirein;
    // tis.deviceid = auth_ti.token_deviceid;
    tis.deviceid = cyberdog_sn;
    tis.state = (is_authenticated == true) ? 0 : -1;
    res.cmd = req.cmd;
    res.data = xpack::json::encode(tis);
  } else if (req.cmd == GET_PARAM) {
    // 获取主控板的参数信息
    cyberdog_param_info cpi;
    cpi.sn = cyberdog_sn;
    cpi.mac = triple_mac;
    cpi.did = triple_did;
    cpi.key = triple_key;
    res.data = xpack::json::encode(cpi);
  } else if (req.cmd == GET_DOG) {
    // 机器狗的个狗信息获取
    dog_info di;
    di.name = personal_info.name;
    di.activate_date = personal_info.activate_date;
    di.weight = personal_info.weight;
    res.data = xpack::json::encode(di);
  } else if (req.cmd == GET_DEVICE_STATUS) {
    get_device_status ges;
    ges.battery_capacity = battery_capicity_;
    res.data = xpack::json::encode(ges);
  }
}

// bool cyberdog::interaction::CyberdogAudio::ClientRequest(
//   const audio_lcm::lcm_data & req,
//   audio_lcm::lcm_data & res)
// {
//   bool result = false;
//   try {
//     if ((check_state < SelfCheckState::SCSNormal) && (req.cmd != SELF_CHECK)) {
//       ERROR(
//         "audio board in %s state.ignore client request:%s.",
//         audio_state.GetSelfCheckState().c_str(), req.cmd.c_str());
//       return false;
//     }
//     LcmClient client(CTOA_SERVICE);
//     result = client.Request(req, res, 1000);
//     if (!result) {
//       ERROR("client thread failed %d times", ++failed_times);
//     } else {
//       DEBUG("client receive: cmd[%s], data[%s]", res.cmd.c_str(), res.data.c_str());
//       if (req.cmd != res.cmd) {
//         ERROR(
//           "client request command:%s, reponse command:%s,data:%s",
//           req.cmd.c_str(), res.cmd.c_str(), res.data.c_str());
//       }
//       if (cyberdog_audio_service_return_cmd_map.find(res.cmd) !=
//         cyberdog_audio_service_return_cmd_map.end())
//       {
//         result = cyberdog_audio_service_return_cmd_map[res.cmd](res.data);
//         if (!result) {
//           INFO("client request return false! cmd:%s, data:%s",
//             res.cmd.c_str(), res.data.c_str());
//         }
//       } else if (res.cmd == BIND_MISERVER) {
//         // bind_server_return bsr;
//         // xpack::json::decode(res.data, bsr);
//         // return bsr.result;
//       } else {
//         WARN(
//           "client reques(%s:%s) unimplement response cmd:%s, data:%s",
//           req.cmd.c_str(), req.data.c_str(),
//           res.cmd.c_str(), res.data.c_str());
//       }
//     }
//   } catch (const std::exception & e) {
//     std::cerr << CTOA_SERVICE << ":" << e.what() << '\n';
//   } catch (...) {
//     WARN("%s:unkown exception", CTOA_SERVICE);
//   }
//   return result;
// }

bool cyberdog::interaction::CyberdogAudio::ClientRequest(
  const audio_lcm::lcm_data & req,
  audio_lcm::lcm_data & res)
{
  if ((check_state < SelfCheckState::SCSNormal) && (req.cmd != SELF_CHECK)) {
    ERROR(
      "audio board in %s state.ignore client request:%s.",
      audio_state.GetSelfCheckState().c_str(), req.cmd.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lck(lcm_ctoa_service_mtx_);
    if (lcm_ctoa_service_ != nullptr) {
      return lcm_ctoa_service_->ClientRequest(req, res);
    }
  }
  return false;
}

bool cyberdog::interaction::CyberdogAudio::ClientRequest2(
  const audio_lcm::lcm_data & req,
  audio_lcm::lcm_data & res)
{
  bool result = true;
  INFO_MILLSECONDS(
    5000,
    "client receive: cmd[%s], data[%s]", res.cmd.c_str(), res.data.c_str());
  if (req.cmd != res.cmd) {
    ERROR(
      "client request command:%s, reponse command:%s,data:%s",
      req.cmd.c_str(), res.cmd.c_str(), res.data.c_str());
  }
  if (cyberdog_audio_service_return_cmd_map.find(res.cmd) !=
    cyberdog_audio_service_return_cmd_map.end())
  {
    result = cyberdog_audio_service_return_cmd_map[res.cmd](res.data);
    if (!result) {
      INFO("client request return false! cmd:%s, data:%s", res.cmd.c_str(), res.data.c_str());
    }
  } else if (res.cmd == BIND_MISERVER) {
    // bind_server_return bsr;
    // xpack::json::decode(res.data, bsr);
    // return bsr.result;
  } else {
    WARN(
      "client request(%s:%s) unimplement response cmd:%s, data:%s",
      req.cmd.c_str(), req.data.c_str(),
      res.cmd.c_str(), res.data.c_str());
  }
  return result;
}
bool cyberdog::interaction::CyberdogAudio::SetControlState(bool on)
{
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_CONTROL_STATE;
  set_control_state set_control_;
  set_control_.on = on;
  req.data = xpack::json::encode(set_control_);
  INFO("set_control_state request:%s", req.data.c_str());
  bool result = ClientRequest(req, res);
  return result;
}
bool cyberdog::interaction::CyberdogAudio::SelfCheck()
{
  // 开机自检
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SELF_CHECK;
  static int32_t counter = 0;
  self_check sc;
  sc.counter = counter++;
  req.data = xpack::json::encode(sc);
  INFO("self check request:%s", req.data.c_str());
  bool result = ClientRequest(req, res);
  return result;
}
int32_t cyberdog::interaction::CyberdogAudio::GetPlayStatus()
{
  bool result = true;
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = GET_PLAY_STATUS;
  result = ClientRequest(req, res);
  if (!result) {
    ERROR("get play status request failed!");
  }
  return audio_play_ptr_->GetPlayStatus();
}
bool cyberdog::interaction::CyberdogAudio::GetPlayStatusResponse(
  const std::string & data)
{
  play_status play_status_res;
  xpack::json::decode(data, play_status_res);
  audio_play_ptr_->SetPlayStatus(play_status_res.status);
  INFO(
    "get_play_status status:%d, volumn:%d, type:%d",
    play_status_res.status, play_status_res.volumn, play_status_res.type);
  return true;
}
bool cyberdog::interaction::CyberdogAudio::SetStatus(uint8_t status)
{
  // 语音状态设置
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_STATUS;
  set_status ss;
  ss.status = status;
  req.data = xpack::json::encode(ss);
  bool result = ClientRequest(req, res);
  return result;
}

bool cyberdog::interaction::CyberdogAudio::SetStatusResponse(
  const std::string & data)
{
  // 语音状态设置返回
  set_status_response ssr;
  xpack::json::decode(data, ssr);
  return ssr.result;
}

bool cyberdog::interaction::CyberdogAudio::GetStatus()
{
  // 语音状态获取
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = GET_STATUS;
  bool result = ClientRequest(req, res);
  return result;
}

bool cyberdog::interaction::CyberdogAudio::GetStatusResponse(
  const std::string & data)
{
  // 获取语音状态返回
  get_status_response gsr;
  xpack::json::decode(data, gsr);
  audio_state.GetAudioWorkState((AudioWorkState)(gsr.status));
  return true;
}

bool cyberdog::interaction::CyberdogAudio::BindServer(
  const uint64_t & uid,
  const std::string & bind_key)
{
  // 音箱绑定米家服务器
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = BIND_MISERVER;
  bind_server bs;
  bs.id = uid;
  bs.key = bind_key;
  req.data = xpack::json::encode(bs);
  bool result = ClientRequest(req, res);
  return result;
}

void cyberdog::interaction::CyberdogAudio::NetStatusNotify()
{
  // 网络连接广播
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  net_status status;
  status.connected = is_wifi_connected;
  l_d->cmd = NETSTATUS;
  l_d->data = xpack::json::encode(status);
  LcmPublish(l_d);
}

bool cyberdog::interaction::CyberdogAudio::SetVolume(uint8_t value)
{
  // 语音状态设置
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_VOLUME;
  set_volume sv;
  sv.value = value;
  req.data = xpack::json::encode(sv);
  INFO("set volume value:%d", value);
  bool result = ClientRequest(req, res);
  if (result) {
    audio_state.SetSoundVolume(value);
  }
  return result;
}

bool cyberdog::interaction::CyberdogAudio::SetVolumeResponse(
  const std::string & data)
{
  // 设置音量返回
  set_volume_response svr;
  xpack::json::decode(data, svr);
  INFO("set volume response:%s", (svr.success == true ? "true" : "false"));
  return svr.success;
}

int8_t cyberdog::interaction::CyberdogAudio::GetVolume()
{
  bool result = true;
  // if (audio_state.GetSoundVolume() < 0) {
  if (true) {
    audio_lcm::lcm_data req;
    audio_lcm::lcm_data res;
    req.cmd = GET_VOLUME;
    result = ClientRequest(req, res);
    if (!result) {
      ERROR("get sound volume request failed!");
    }
  }
  INFO("get volume value:%d", audio_state.GetSoundVolume());
  return audio_state.GetSoundVolume();
}

bool cyberdog::interaction::CyberdogAudio::GetVolumeResponse(
  const std::string & data)
{
  // 设置音量返回
  get_volume_response gvr;
  xpack::json::decode(data, gvr);
  audio_state.SetSoundVolume(gvr.value);
  return true;
}

void cyberdog::interaction::CyberdogAudio::BindStatus(const std::string & data)
{
  // 通知绑定米家服务器结果
  bind_status_data bsd;
  xpack::json::decode(data, bsd);
}

void cyberdog::interaction::CyberdogAudio::VoiceprintResult(const std::string & data)
{
  // 录制声纹结果通知
  voiceprint_result vr;
  xpack::json::decode(data, vr);
  protocol::msg::AudioVoiceprintResult avr;
  avr.code = vr.code;
  avr.voice_print.id = vr.id;
  audio_voiceprint_result_pub_->publish(avr);
  INFO("voiceprint train id:%s, result code:%d", vr.id.c_str(), vr.code);
  if (avr.voice_print.id == voice_print_training_info_.id) {
    if (vr.code == 0) {
      vp_database_ptr_->Insert(voice_print_training_info_);
      account_manager_ptr_->ModifyUserInformation(voice_print_training_info_.name, 1, 0);
    } else {
      account_manager_ptr_->ModifyUserInformation(voice_print_training_info_.name, 0, 0);
    }
    INFO("account modify information success");
  } else {
    WARN(
      "name:%s, real id:%s, notify id:%s", voice_print_training_info_.name.c_str(),
      voice_print_training_info_.id.c_str(), avr.voice_print.id.c_str());
  }
}

// bool cyberdog::interaction::CyberdogAudio::GetMiotDid()
// {
//   // 获取语音板miot_did
//   audio_lcm::lcm_data req;
//   audio_lcm::lcm_data res;
//   req.cmd = MIOT_DID;
//   bool result = ClientRequest(req, res);
//   if (result) {
//     miot_did md;
//     xpack::json::decode(res.data, md);
//     // audio_miot_did = md.id;
//   }
//   return result;
// }

void cyberdog::interaction::CyberdogAudio::AccountTokenNotify()
{
  // 小米账号token通知
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  token_info ti;
  ti.uid = auth_ti.uid;
  ti.access = auth_ti.token_access;
  ti.fresh = auth_ti.token_fresh;
  ti.expirein = auth_ti.token_expirein;
  // ti.deviceid = audio_miot_did;
  // ti.deviceid = triple_did;
  ti.deviceid = cyberdog_sn;
  l_d->cmd = ACCOUNT_TOKEN;
  l_d->data = xpack::json::encode(ti);
  LcmPublish(l_d);
}

bool cyberdog::interaction::CyberdogAudio::SetToken()
{
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_TOKEN;
  token_info ti;
  ti.uid = auth_ti.uid;
  ti.access = auth_ti.token_access;
  ti.fresh = auth_ti.token_fresh;
  ti.expirein = auth_ti.token_expirein;
  // ti.deviceid = triple_did;
  ti.deviceid = cyberdog_sn;
  req.data = xpack::json::encode(ti);
  bool result = ClientRequest(req, res);
  return result;
}

bool cyberdog::interaction::CyberdogAudio::SetTokenResponse(const std::string & data)
{
  set_token_response str;
  xpack::json::decode(data, str);
  return str.result;
}

bool cyberdog::interaction::CyberdogAudio::SetWake(const std::string & word)
{
  // 设置机器狗唤醒词
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_WAKE;
  wake_word ww;
  ww.word = word;
  req.data = xpack::json::encode(ww);
  bool result = ClientRequest(req, res);
  return result;
}

bool cyberdog::interaction::CyberdogAudio::SetWakeResponse(const std::string & data)
{
  set_wake_response swr;
  xpack::json::decode(data, swr);
  return swr.result;
}

void cyberdog::interaction::CyberdogAudio::DogInfoNotify()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  dog_info di;
  di.name = personal_info.name;
  di.activate_date = personal_info.activate_date;
  di.weight = personal_info.weight;
  l_d->cmd = DOG_INFO;
  l_d->data = xpack::json::encode(di);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::VerRequestNotify()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = VERSION_REQUEST;
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::OtaRequestNotify()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = OTA_REQUEST;
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::OtaOverNotify()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  ota_over oo;
  oo.reboot = true;
  l_d->cmd = OTA_OVER;
  l_d->data = xpack::json::encode(oo);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::VoiceprintRegisterNotify(std::string & name)
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  voiceprint_register vr;
  vr.name = name;
  l_d->cmd = VOICEPRINT_REGISTER;
  l_d->data = xpack::json::encode(vr);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::VoiceprintCancelNotify()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  voiceprint_cancel vc;
  vc.prompt = "已退出声纹注册流程，可以到APP端的家庭成员设置中重新开始注册";
  l_d->cmd = VOICEPRINT_CANCEL;
  l_d->data = xpack::json::encode(vc);
  LcmPublish(l_d);
}

void cyberdog::interaction::CyberdogAudio::VersionResponse(const std::string & data)
{
  // 版本查询应答
  version_response vr;
  xpack::json::decode(data, vr);
  ota_data_version odv;
  odv.version = vr.version;
  std::string params = xpack::json::encode(odv);
  ota_data od;
  od.command = VERSION_REQUEST_CODE;
  od.params = params;
  std_msgs::msg::String msg;
  msg.data = xpack::json::encode(od);
  ota_response_pub_->publish(msg);
}

void cyberdog::interaction::CyberdogAudio::OtaResponse(const std::string & data)
{
  // ota应答通知
  ota_response ore;
  xpack::json::decode(data, ore);
  if (ore.ready) {
    ota_data od;
    od.command = OTA_REQUEST_CODE;
    std_msgs::msg::String msg;
    msg.data = xpack::json::encode(od);
    ota_response_pub_->publish(msg);
  } else {
    WARN("ota response not ready");
  }
}

void cyberdog::interaction::CyberdogAudio::RestoreSettings()
{
  std::shared_ptr<audio_lcm::lcm_data> l_d(new audio_lcm::lcm_data());
  l_d->cmd = RESTORE_SETTINGS;
  LcmPublish(l_d);
}

bool cyberdog::interaction::CyberdogAudio::SetEnvironment()
{
  // 设置R329线上环境
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = SET_ENV;
  set_env se;
  // se.environment = "preview4test";
  se.environment = audio_environment_;
  req.data = xpack::json::encode(se);
  bool result = ClientRequest(req, res);
  INFO(
    "set environment:%s, result:%s",
    req.data.c_str(),
    (result ? "true" : "false"));
  return result;
}

bool cyberdog::interaction::CyberdogAudio::SetEnvironmentResponse(const std::string & data)
{
  if (!data.empty()) {
    set_env_response ser;
    xpack::json::decode(data, ser);
    INFO("set environment response:%s", (ser.success == true ? "true" : "false"));
    return ser.success;
  }
  return false;
}

bool cyberdog::interaction::CyberdogAudio::GetEnvironment()
{
  // 获取R329线上环境
  audio_lcm::lcm_data req;
  audio_lcm::lcm_data res;
  req.cmd = GET_ENV;
  bool result = ClientRequest(req, res);
  return result;
}

bool cyberdog::interaction::CyberdogAudio::GetEnvironmentResponse(const std::string & data)
{
  if (!data.empty()) {
    get_env_response ger;
    xpack::json::decode(data, ger);
    INFO("get environment:%s from audio board", ger.environment.c_str());
    return ger.environment == audio_environment_ ? true : false;
  }
  return false;
}

void cyberdog::interaction::CyberdogAudio::Wakeup(const std::string & data)
{
  voice_control_ptr_->Wakeup(data);
  std_msgs::msg::Bool msg;
  msg.data = true;
  wake_up_pub_->publish(msg);
}

void cyberdog::interaction::CyberdogAudio::RegisterAudioCyberdogTopicHandler()
{
  // 本地语音控制指令
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      LOCAL_CONTROL,
      std::bind(&VoiceControl::Local, voice_control_ptr_, std::placeholders::_1)));
  // 在线语音控制指令
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      ONLINE_CONTROL,
      std::bind(&VoiceControl::Online, voice_control_ptr_, std::placeholders::_1)));
  // 唤醒寻向通知
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      WAKE_UP,
      std::bind(&CyberdogAudio::Wakeup, this, std::placeholders::_1)));
  // 环境音检测通知
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      ENVIRONMENT_VOICE,
      std::bind(&VoiceControl::Environment, voice_control_ptr_, std::placeholders::_1)));
  // 通知米家服务器绑定结果
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      BIND_STATUS,
      std::bind(&CyberdogAudio::BindStatus, this, std::placeholders::_1)));
  // 录制声纹结果通知
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      VOICEPRINT_RESULT,
      std::bind(&CyberdogAudio::VoiceprintResult, this, std::placeholders::_1)));
  // 版本查询请求
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      VERSION_RESPONSE,
      std::bind(&CyberdogAudio::VersionResponse, this, std::placeholders::_1)));
  // ota请求通知
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      OTA_RESPONSE,
      std::bind(&CyberdogAudio::OtaResponse, this, std::placeholders::_1)));
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      PLAY_NOTIFY,
      std::bind(&AudioPlay::PlayNotify, audio_play_ptr_, std::placeholders::_1)));
  audio_cyberdog_topic_cmd_map.insert(
    std::make_pair(
      HTTP_PLAY_NOTIFY,
      std::bind(&AudioPlay::HttpPlayNotify, audio_play_ptr_, std::placeholders::_1)));
}

void cyberdog::interaction::CyberdogAudio::RegisterCyberdogAudioServiceReturnHandler()
{
  // 自检返回
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      GET_PLAY_STATUS,
      std::bind(&CyberdogAudio::GetPlayStatusResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SELF_CHECK,
      std::bind(&AudioState::SetSelfCheckState, &audio_state, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SET_STATUS,
      std::bind(&CyberdogAudio::SetStatusResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      GET_STATUS,
      std::bind(&CyberdogAudio::GetStatusResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SET_WAKE,
      std::bind(&CyberdogAudio::SetWakeResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SET_TOKEN,
      std::bind(&CyberdogAudio::SetTokenResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      GET_VOLUME,
      std::bind(&CyberdogAudio::GetVolumeResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SET_VOLUME,
      std::bind(&CyberdogAudio::SetVolumeResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      SET_ENV,
      std::bind(&CyberdogAudio::SetEnvironmentResponse, this, std::placeholders::_1)));
  cyberdog_audio_service_return_cmd_map.insert(
    std::make_pair(
      GET_ENV,
      std::bind(&CyberdogAudio::GetEnvironmentResponse, this, std::placeholders::_1)));
}

void cyberdog::interaction::CyberdogAudio::RegisterNotify(SelfCheckState & code)
{
  static bool ai_enable = true;
  check_state = code;
  if (check_state == SelfCheckState::SCSUnBindServer) {
    auto msg = std::make_shared<protocol::msg::AudioPlay>();
    msg->module_name = "self";
    msg->play_id = protocol::msg::AudioPlay::PID_AI_PLEASE_ENABLE;
    SpeechCallback(msg);
    ai_enable = false;
  } else if (!ai_enable && (check_state == SelfCheckState::SCSNormal) ) {
    // auto msg = std::make_shared<protocol::msg::AudioPlay>();
    // msg->module_name = "self";
    // msg->play_id = protocol::msg::AudioPlay::PID_AI_ENABLE_SUCCESS;
    // SpeechCallback(msg);
    // ai_enable = true;
  }
}
