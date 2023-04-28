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

#include <string>
#include <vector>

#include "cyberdog_vp_terminal/common.hpp"

namespace cyberdog_visual_programming_terminal
{
std::string GetTime(int nowModo)
{
  std::string mapBackupsTime;
  if (nowModo == static_cast<int>(TimeMode::Ms1970)) {
    timeval tv;
    gettimeofday(&tv, 0);
    int64_t _mapBackupsTime = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;
    std::stringstream time_u_sec;
    time_u_sec << _mapBackupsTime;
    time_u_sec >> mapBackupsTime;
    time_u_sec.clear();
    time_u_sec.str("");
  } else if (nowModo == static_cast<int>(TimeMode::_Y_M_D_H_M_S)) {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "T%YY%mM%dD%HH%MM%SS", localtime(&timep));
    mapBackupsTime = tmp;
  } else if (nowModo == static_cast<int>(TimeMode::STANDARD)) {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y.%m.%d-%H:%M:%S", localtime(&timep));
    mapBackupsTime = tmp;
  }
  return mapBackupsTime;
}

bool JudgeConfileFile(std::string _file)
{
  DEBUG("Judge params config file dir:%s.", _file.c_str());
  if (access(_file.c_str(), F_OK)) {
    ERROR("Toml config file does not exist, config file dir:\n%s", _file.c_str());
    return false;
  }
  if (access(_file.c_str(), R_OK)) {
    ERROR("Toml config file does not have read permissions, config file dir:\n%s", _file.c_str());
    return false;
  }
  if (access(_file.c_str(), W_OK)) {
    ERROR("Toml config file does not have write permissions, config file dir:\n%s", _file.c_str());
    return false;
  }
  return true;
}

std::vector<std::string> GetVector(
  const std::string & _message, char _delim,
  const std::string & _head)
{
  std::vector<std::string> _vector;
  std::stringstream message_str;
  message_str.str(_message);
  std::string elems;
  while (std::getline(message_str, elems, _delim)) {
    _vector.push_back(std::string(_head + elems));
  }
  return _vector;
}

void CoutJson(const std::string & _msg_head, const std::string & _msg)
{
  try {
    rapidjson::Document doc;
    if (cyberdog::common::CyberdogJson::String2Document(_msg, doc)) {
      rapidjson::StringBuffer buffer;
      rapidjson::PrettyWriter<rapidjson::StringBuffer> pretty_writer(buffer);
      pretty_writer.SetMaxDecimalPlaces(4);
      doc.Accept(pretty_writer);
      DEBUG("%s :\n%s", _msg_head.c_str(), buffer.GetString());
    }
  } catch (const std::exception & e) {
    WARN("Error responding to message:\nmessage:%s\nerror:%s", _msg.c_str(), e.what());
  }
}
}   // namespace cyberdog_visual_programming_terminal
