// Copyright (c) 2022  Xiaomi Corporation
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

#include "examples/peerconnection/client/defaults.h"

#include <stdlib.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#endif

#include "rtc_base/arraysize.h"

const char kAudioLabel[] = "audio_label";
const char kVideoLabel[] = "video_label";
const char kStreamId[] = "stream_id";
const uint16_t kDefaultServerPort = 8888;

std::string GetEnvVarOrDefault(
  const char * env_var_name,
  const char * default_value)
{
  std::string value;
  const char * env_var = getenv(env_var_name);
  if (env_var) {
    value = env_var;
  }

  if (value.empty()) {
    value = default_value;
  }

  return value;
}

std::string GetPeerConnectionString()
{
  return GetEnvVarOrDefault("WEBRTC_CONNECT", "stun:stun.l.google.com:19302");
}

std::string GetDefaultServerName()
{
  return GetEnvVarOrDefault("WEBRTC_SERVER", "localhost");
}

std::string GetPeerName()
{
  char computer_name[256];
  std::string ret(GetEnvVarOrDefault("USERNAME", "user"));
  ret += '@';
  if (gethostname(computer_name, arraysize(computer_name)) == 0) {
    ret += computer_name;
  } else {
    ret += "host";
  }
  return ret;
}
