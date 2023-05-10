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
#ifndef CYBERDOG_AUDIO__CYBERDOG_AUDIO_HPP_
#define CYBERDOG_AUDIO__CYBERDOG_AUDIO_HPP_

#include <string>
#include <thread>
#include <memory>
#include <map>
#include <mutex>
#include <condition_variable>
// #include <lcm/lcm-cpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/wifi_status.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/msg/audio_voiceprint_result.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/auth_token.hpp"
#include "protocol/msg/train_plan.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "protocol/srv/audio_auth_id.hpp"
#include "protocol/srv/audio_auth_token.hpp"
#include "protocol/srv/audio_voiceprint_train.hpp"
#include "protocol/srv/audio_voiceprints_set.hpp"
#include "protocol/srv/audio_nick_name.hpp"
#include "protocol/srv/audio_volume_set.hpp"
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/srv/audio_voiceprint_entry.hpp"
#include "protocol/srv/trigger.hpp"
#include "protocol/srv/sdcard_play_id_query.hpp"
#include "protocol/srv/train_plan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "cyberdog_audio/speech_handler.hpp"
#include "cyberdog_audio/voice_control.hpp"
#include "cyberdog_audio/audio_state.hpp"
#include "cyberdog_audio/voiceprint_database.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"
#include "user_info_manager/UserAccountManager.hpp"
#include "cyberdog_audio/machine_state.hpp"
#include "cyberdog_audio/lcm_atoc_service.hpp"
#include "cyberdog_audio/lcm_atoc_topic.hpp"
#include "cyberdog_audio/lcm_ctoa_service.hpp"
#include "cyberdog_audio/lcm_ctoa_topic.hpp"
#include "cyberdog_audio/parameters_info.hpp"
#include "cyberdog_audio/ready_sn.hpp"
#include "cyberdog_audio/audio_play.hpp"
#include "cyberdog_audio/audio_fds.hpp"

#define   NODE_NAME   "cyberdog_audio"

namespace cyberdog
{
namespace interaction
{
using LcmServer = cyberdog::common::LcmServer<audio_lcm::lcm_data, audio_lcm::lcm_data>;
using LcmClient = cyberdog::common::LcmClient<audio_lcm::lcm_data, audio_lcm::lcm_data>;

using AUDIO_CYBERDOG_TOPIC_FUNC = std::function<void (const std::string &)>;
using CYBERDOG_AUDIO_SERVICE_RETURN_FUNC = std::function<bool (const std::string &)>;

enum class AudioErrorCode : int32_t
{
  kAudioMicOnOffError = 21,
  kAudioMicStateError = 22,
  kAudioSetTokenError = 23,
  kAudioVoiceprintTrainError = 24,
  kAudioSetNickNameError = 25,
  kAudioSetVoiceError = 26,
  kAudioGetVoiceError = 27,
  kAudioControlVerticalDomainEnableError = 28,
  kAudioGetControlVerticalDomainError = 29,
  kAudioSwitchEnvironmentError = 30,
};

class CyberdogAudio final : public rclcpp::Node
{
public:
  // explicit CyberdogAudio(std::string & sn);
  CyberdogAudio();
  ~CyberdogAudio();
  void SpeechPlayGoal(const std::shared_ptr<SpeechActionServer::GoalHandleSpeech> goal_handle);

private:
  void CreateLcm();
  void DestroyLcm();
  int32_t OnSetUp();
  int32_t OnSelfCheck();
  int32_t OnActive();
  int32_t OnDeactive();
  int32_t OnTearDown();
  int32_t OnLowPower();
  int32_t OnOta();
  void SpeechCallback(const protocol::msg::AudioPlay::SharedPtr msg);
  void SpeechExtendCallback(const protocol::msg::AudioPlayExtend::SharedPtr msg);
  void WifiCallback(const protocol::msg::WifiStatus::SharedPtr msg);
  void WakeWordCallback(const std_msgs::msg::String::SharedPtr msg);
  void DogInfoCallback(const std_msgs::msg::String::SharedPtr msg);
  void OtaRequestCallback(const std_msgs::msg::String::SharedPtr msg);
  void VolumeSetCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void VolumeGetCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void RestoreSettingsCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg);
  void ContinueDialog(const std_msgs::msg::Bool::SharedPtr msg);
  void SetAudioState(
    const protocol::srv::AudioExecute::Request::SharedPtr request,
    protocol::srv::AudioExecute::Response::SharedPtr respose);
  void GetAudioState(
    const protocol::srv::AudioExecute::Request::SharedPtr request,
    protocol::srv::AudioExecute::Response::SharedPtr respose);
  void GetAuthDid(
    const protocol::srv::AudioAuthId::Request::SharedPtr,
    protocol::srv::AudioAuthId::Response::SharedPtr response);
  void SetAuthToken(
    const protocol::srv::AudioAuthToken::Request::SharedPtr request,
    protocol::srv::AudioAuthToken::Response::SharedPtr response);
  void VoiceprintCallback(
    const protocol::srv::AudioVoiceprintTrain::Request::SharedPtr request,
    protocol::srv::AudioVoiceprintTrain::Response::SharedPtr);
  void VoiceprintsSetCallback(
    const protocol::srv::AudioVoiceprintsSet::Request::SharedPtr request,
    protocol::srv::AudioVoiceprintsSet::Response::SharedPtr response);
  void SetNickNameSwitchCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void SetNickNameCallback(
    const protocol::srv::AudioNickName::Request::SharedPtr request,
    protocol::srv::AudioNickName::Response::SharedPtr response);
  void AudioVolumeSet(
    const protocol::srv::AudioVolumeSet::Request::SharedPtr request,
    protocol::srv::AudioVolumeSet::Response::SharedPtr response);
  void AudioVolumeGet(
    const protocol::srv::AudioVolumeGet::Request::SharedPtr request,
    protocol::srv::AudioVolumeGet::Response::SharedPtr response);
  void AudioTextPlay(
    const protocol::srv::AudioTextPlay::Request::SharedPtr request,
    protocol::srv::AudioTextPlay::Response::SharedPtr response);
  void SdcardPlayidQuery(
    const protocol::srv::SdcardPlayIdQuery::Request::SharedPtr request,
    protocol::srv::SdcardPlayIdQuery::Response::SharedPtr response);
  void AudiActiveState(
    const protocol::srv::Trigger::Request::SharedPtr request,
    protocol::srv::Trigger::Response::SharedPtr response);
  void AudioActionSetCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void AudioActionGetCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void AudioVoiceEntry(
    const protocol::srv::AudioVoiceprintEntry::Request::SharedPtr request,
    protocol::srv::AudioVoiceprintEntry::Response::SharedPtr response);
  void SwitchEnvironmentService(
    const protocol::srv::Trigger::Request::SharedPtr request,
    protocol::srv::Trigger::Response::SharedPtr response);
  void PowerTimerCallback();
  void RebootTimerCallback();
  void LcmHandler(
    const lcm::ReceiveBuffer * rbuf, const std::string & chan,
    const audio_lcm::lcm_data * msg);
  // void LcmPublish(const std::shared_ptr<audio_lcm::lcm_data> & data);
  void LcmPublish(const std::shared_ptr<audio_lcm::lcm_data> & data);
  int32_t LcmPublish2(const std::shared_ptr<audio_lcm::lcm_data> & data);
  void ServerCallback(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res);
  bool ClientRequest(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res);
  bool ClientRequest2(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res);
  bool SelfCheck();
  bool SetStatus(uint8_t status);
  bool SetStatusResponse(const std::string & data);
  bool GetStatus();
  bool GetStatusResponse(const std::string & data);
  bool BindServer(const uint64_t & uid, const std::string & bind_key);
  void BindStatus(const std::string & data);
  void VoiceprintResult(const std::string & data);
  void NetStatusNotify();
  bool SetVolume(uint8_t value);
  bool SetVolumeResponse(const std::string & data);
  int8_t GetVolume();
  bool GetVolumeResponse(const std::string & data);
  // bool GetMiotDid();
  void AccountTokenNotify();
  bool SetToken();
  bool SetTokenResponse(const std::string & data);
  bool SetWake(const std::string & word);
  bool SetWakeResponse(const std::string & data);
  void DogInfoNotify();
  void VerRequestNotify();
  void OtaRequestNotify();
  void OtaOverNotify();
  void VoiceprintRegisterNotify(std::string & name);
  void VoiceprintCancelNotify();
  void VersionResponse(const std::string & data);
  void OtaResponse(const std::string & data);
  void RestoreSettings();
  bool SetEnvironment();
  bool SetEnvironmentResponse(const std::string & data);
  bool GetEnvironment();
  bool GetEnvironmentResponse(const std::string & data);
  void Wakeup(const std::string & data);

private:
  void RegisterAudioCyberdogTopicHandler();
  void RegisterCyberdogAudioServiceReturnHandler();
  void RegisterNotify(SelfCheckState & code);

private:
  // std::shared_ptr<lcm::LCM> lcm_;
  // std::shared_ptr<LcmServer> server;
  // std::thread server_thread;
  // std::thread message_thread;
  std::unique_ptr<ReadySnNode> ready_sn_ptr {nullptr};
  std::unique_ptr<LcmCtoaTopic> lcm_ctoa_topic_ {nullptr};
  std::unique_ptr<LcmAtocTopic> lcm_atoc_topic_ {nullptr};
  std::unique_ptr<LcmCtoaService> lcm_ctoa_service_ {nullptr};
  std::unique_ptr<LcmAtocService> lcm_atoc_service_ {nullptr};
  std::thread heart_thread;
  rclcpp::Subscription<protocol::msg::AudioPlay>::SharedPtr speech_sub_;
  rclcpp::Subscription<protocol::msg::AudioPlayExtend>::SharedPtr speech_extend_sub_;
  rclcpp::Subscription<protocol::msg::WifiStatus>::SharedPtr wifi_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wake_word_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dog_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ota_request_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr volume_set_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr volume_get_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr restore_settings_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr continue_dialog_sub_;
  rclcpp::Service<protocol::srv::AudioExecute>::SharedPtr audio_set_status_srv_;
  rclcpp::Service<protocol::srv::AudioExecute>::SharedPtr audio_get_status_srv_;
  rclcpp::Service<protocol::srv::AudioAuthId>::SharedPtr audio_auth_did_srv_;
  rclcpp::Service<protocol::srv::AudioAuthToken>::SharedPtr audio_auth_token_srv_;
  rclcpp::Service<protocol::srv::AudioVoiceprintTrain>::SharedPtr audio_voiceprint_train_srv_;
  rclcpp::Service<protocol::srv::AudioVoiceprintsSet>::SharedPtr audio_voiceprints_data_set_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr audio_nick_name_switch_srv_;
  rclcpp::Service<protocol::srv::AudioNickName>::SharedPtr audio_nick_name_set_srv_;
  rclcpp::Service<protocol::srv::AudioVolumeSet>::SharedPtr audio_volume_set_srv_;
  rclcpp::Service<protocol::srv::AudioVolumeGet>::SharedPtr audio_volume_get_srv_;
  rclcpp::Service<protocol::srv::AudioTextPlay>::SharedPtr audio_text_play_srv_;
  rclcpp::Service<protocol::srv::Trigger>::SharedPtr audio_active_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr audio_action_set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr audio_action_get_srv_;
  rclcpp::Service<protocol::srv::SdcardPlayIdQuery>::SharedPtr sdcard_playid_query_srv_;
  rclcpp::Service<protocol::srv::AudioVoiceprintEntry>::SharedPtr audio_voiceprint_entry_srv_;
  rclcpp::Service<protocol::srv::Trigger>::SharedPtr switch_environment_srv_;
  rclcpp::Publisher<protocol::msg::AudioVoiceprintResult>::SharedPtr audio_voiceprint_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_voiceprints_data_get_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ota_response_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_asr_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dog_info_update_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wake_up_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aiconnect_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr audio_board_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dlg_info_pub_;
  rclcpp::Publisher<protocol::msg::AuthToken>::SharedPtr audio_auth_token_pub_;
  rclcpp::Publisher<protocol::msg::TrainPlan>::SharedPtr train_plan_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reboot_client_;
  rclcpp::Client<protocol::srv::TrainPlan>::SharedPtr train_plan_client_;
  rclcpp::TimerBase::SharedPtr power_timer_;
  // std::shared_ptr<SpeechHandler> speech_handler_ptr_;
  std::shared_ptr<AudioPlay> audio_play_ptr_;
  std::shared_ptr<VoiceControl> voice_control_ptr_;
  uint32_t recv_msg_cnt;
  AudioState audio_state;
  // bool is_audio_normal;
  SelfCheckState check_state;
  std::atomic_bool is_wifi_connected;
  bool first_notified_net;
  uint32_t failed_times;
  // std::string audio_miot_did;
  std::string cyberdog_sn;
  std::string triple_mac;
  std::string triple_did;
  std::string triple_key;
  bool is_authenticated;
  std::string auth_uid;
  TokenInfo auth_ti;
  DogInfo personal_info;
  std::map<const std::string, AUDIO_CYBERDOG_TOPIC_FUNC> audio_cyberdog_topic_cmd_map;
  std::map<const std::string,
    CYBERDOG_AUDIO_SERVICE_RETURN_FUNC> cyberdog_audio_service_return_cmd_map;
  std::unique_ptr<cyberdog::machine::HeartBeatsActuator> heart_beats_ptr_;
  std::unique_ptr<cyberdog::common::CyberdogAccountManager> account_manager_ptr_;
  std::unique_ptr<cyberdog::interaction::VoiceprintDatabase> vp_database_ptr_;
  std::unique_ptr<cyberdog::interaction::MachineState> machine_state_ptr_;
  std::unique_ptr<cyberdog::interaction::AudioFds> audio_fds_ptr_;
  // std::mutex play_mtx_;
  // bool is_play_;
  // std::condition_variable play_cv_;
  bool token_invalid_;
  VoicePrint voice_print_training_info_;
  bool switch_environment_;
  std::string audio_environment_;
  bool action_control_enable_;
  std::mutex lcm_ctoa_topic_mtx_;
  std::mutex lcm_atoc_topic_mtx_;
  std::mutex lcm_ctoa_service_mtx_;
  std::mutex lcm_atoc_service_mtx_;
  bool is_active_;
  uint8_t battery_capicity_;
  std::shared_ptr<cyberdog::system::CyberdogCode<AudioErrorCode>> code_ptr_ {nullptr};

  const float CONST_WEIGHT = 7.6;

  rclcpp::CallbackGroup::SharedPtr speech_callback_group_;
  rclcpp::CallbackGroup::SharedPtr volumn_callback_group_;
  rclcpp::CallbackGroup::SharedPtr bc_callback_group_;
  rclcpp::CallbackGroup::SharedPtr power_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr train_callback_group_;
};      // CyberdogAudio
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__CYBERDOG_AUDIO_HPP_
