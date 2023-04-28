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
#ifndef CYBERDOG_AUDIO__INSTRUCTION_DEF_HPP_
#define CYBERDOG_AUDIO__INSTRUCTION_DEF_HPP_

#include <string>
#include <vector>
#include "xpack/json.h"

#define  CTOA_TOPIC             "cyberdog_audio_topic"
#define  CTOA_SERVICE           "cyberdog_audio_service"
#define  ATOC_TOPIC             "audio_cyberdog_topic"
#define  ATOC_SERVICE           "audio_cyberdog_service"
#define  SELF_CHECK             "self_check"
#define  PLAY                   "play"
#define  PLAY_CANCEL            "play_cancel"
#define  PLAY_NOTIFY            "play_notify"
#define  HTTP_PLAY_NOTIFY       "http_play_notify"
#define  TTS_PLAY               "tts_play"
#define  HTTP_TTS_PLAY          "http_tts_play"
#define  HTTP_MUSIC_PLAY        "http_music_play"
#define  SET_STATUS             "set_status"
#define  GET_STATUS             "get_status"
#define  GET_NETSTATUS          "get_netstatus"
#define  NETSTATUS              "netstatus"
#define  LOCAL_CONTROL          "local_control"
#define  ONLINE_CONTROL         "online_control"
#define  WAKE_UP                "wake_up"
#define  ENVIRONMENT_VOICE      "environment_voice"
#define  BIND_MISERVER          "bind_miserver"
#define  BIND_STATUS            "bind_status"
#define  MIOT_DID               "miot_did"
#define  ACCOUNT_TOKEN          "account_token"
#define  SET_TOKEN              "set_token"
#define  GET_TOKEN              "get_token"
#define  TOKEN_STATE            "token_state"
#define  AI_INSTRUCTION         "ai_instruction"
#define  GET_PARAM              "get_param"
#define  SET_WAKE               "set_wake"
#define  ASR_NOTIFY             "asr_notify"
#define  DOG_INFO               "dog_info"
#define  GET_DOG                "get_dog"
#define  VERSION_REQUEST        "version_request"
#define  VERSION_RESPONSE       "version_response"
#define  OTA_REQUEST            "ota_request"
#define  OTA_RESPONSE           "ota_response"
#define  OTA_OVER               "ota_over"
#define  VOICEPRINT_REGISTER    "voiceprint_register"
#define  VOICEPRINT_RESULT      "voiceprint_result"
#define  VOICEPRINT_CANCEL      "voiceprint_cancel"
#define  DLG_INFO               "dlg_info"
#define  AI_STATE               "ai_state"
#define  AI_ERROR               "ai_error"
#define  SET_VOLUME             "set_volume"
#define  GET_VOLUME             "get_volume"
#define  RESTORE_SETTINGS       "restore_settings"
#define  SETTINGS_RESTORE_NOTIFY "settings_restore_notify"
#define  SET_ENV                "set_env"
#define  GET_ENV                "get_env"
#define  AUDIO_BOARD_STATE      "audio_board_state"
#define  POWER_INSTRUCTION      "power_instruction"
#define  GET_DEVICE_STATUS      "get_device_status"
#define  PERSONALIZE_INSTRUCTION "personalize_instruction"

struct self_check
{
  int32_t counter;
  XPACK(O(counter))
};
struct get_net_status
{
  bool result;
  XPACK(O(result))
};
struct net_status
{
  bool connected;
  XPACK(O(connected))
};
struct set_status
{
  uint8_t status;
  XPACK(O(status))
};
struct set_status_response
{
  bool result;
  XPACK(O(result))
};
struct get_status_response
{
  uint8_t status;
  XPACK(O(status))
};
struct bind_server
{
  uint64_t id;
  std::string key;
  XPACK(O(id, key))
};
struct bind_server_return
{
  bool result;
  XPACK(O(result))
};
struct bind_status_data
{
  uint64_t id;
  std::string key;
  int code;
  XPACK(O(id, key, code))
};
struct miot_did
{
  std::string id;
  XPACK(O(id))
};
struct token_info
{
  std::string uid;
  std::string access;
  std::string fresh;
  uint64_t expirein;
  std::string deviceid;
  XPACK(O(uid, access, fresh, expirein, deviceid))
};
struct token_info_state
{
  std::string uid;
  std::string access;
  std::string fresh;
  uint64_t expirein;
  std::string deviceid;
  int8_t state;
  XPACK(O(uid, access, fresh, expirein, deviceid, state))
};
struct cyberdog_param_info
{
  std::string sn;
  std::string mac;
  std::string did;
  std::string key;
  XPACK(O(sn, mac, did, key))
};
struct play_notify_info
{
  std::string name;
  uint8_t status;
  XPACK(O(name, status))
};
struct http_play_notify_info
{
  std::string name;
  uint8_t status;
  XPACK(O(name, status))
};
struct wake_word
{
  std::string word;
  XPACK(O(word))
};
struct set_wake_response
{
  bool result;
  XPACK(O(result))
};
struct set_token_response
{
  bool result;
  XPACK(O(result))
};
struct dog_info
{
  std::string name;
  std::string activate_date;
  float weight;
  XPACK(O(name, activate_date, weight))
};
struct version_response
{
  std::string version;
  XPACK(O(version))
};
struct ota_response
{
  bool ready;
  XPACK(O(ready))
};
struct ota_over
{
  bool reboot;
  XPACK(O(reboot))
};
struct voiceprint_register
{
  std::string name;
  XPACK(O(name))
};
struct voiceprint_result
{
  int code;
  std::string id;
  XPACK(O(code, id))
};
struct voiceprint_cancel
{
  std::string prompt;
  XPACK(O(prompt))
};
struct ota_data
{
  uint32_t command;
  std::string params;
  XPACK(O(command, params))
};
struct ota_data_version
{
  std::string major;
  std::string minor;
  std::string patch;
  std::string version;
  XPACK(O(major, minor, patch, version))
};
struct asr_notify
{
  std::string text;
  bool final;
  bool is_online;
  std::string dialogid;
  XPACK(O(text, final, is_online, dialogid))
};
struct dlg_info
{
  std::string dialogid;
  std::string voiceid;
  std::string nickname;
  XPACK(O(dialogid, voiceid, nickname))
};
struct ai_state
{
  bool connected;
  XPACK(O(connected))
};
struct ai_error
{
  int code;
  std::string message;
  std::string eventid;
  XPACK(O(code, message, eventid))
};
struct settings_restore_notify
{
  bool completed;
  XPACK(O(completed))
};
struct audio_board_state
{
  uint8_t state;
  XPACK(O(state))
};
struct power_instruction
{
  std::string operation;
  int delay;
  bool confirmation;
  XPACK(O(operation, delay, confirmation))
};
struct set_volume
{
  uint8_t value;
  XPACK(O(value))
};
struct set_volume_response
{
  bool success;
  XPACK(O(success))
};
struct get_volume_response
{
  uint8_t value;
  XPACK(O(value))
};
struct set_env
{
  std::string environment;
  XPACK(O(environment))
};
struct set_env_response
{
  bool success;
  XPACK(O(success))
};
struct get_env_response
{
  std::string environment;
  XPACK(O(environment))
};
struct get_device_status
{
  int32_t battery_capacity;
  XPACK(O(battery_capacity))
};
struct personalize_instruction_keyval
{
  std::string exectype;
  std::string execvalue;
  XPACK(O(exectype, execvalue))
};
struct personalize_instruction
{
  std::vector<std::string> cmds;
  XPACK(O(cmds))
};
struct TokenInfo
{
  std::string uid;
  std::string token_access;
  std::string token_fresh;
  uint64_t token_expirein;
  std::string token_deviceid;
};
struct DogInfo
{
  std::string name;
  std::string activate_date;
  float weight;
};

#endif  // CYBERDOG_AUDIO__INSTRUCTION_DEF_HPP_
