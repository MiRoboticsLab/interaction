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
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"

namespace cyberdog_visual_programming_abilityset
{
geometry_msgs::msg::Quaternion RPY2Qrientation(
  const double _r,
  const double _p,
  const double _y)
{
  geometry_msgs::msg::Quaternion ret;
  tf2::Quaternion orientation;
  orientation.setRPY(_r, _p, _y);
  ret.x = orientation.x();
  ret.y = orientation.y();
  ret.z = orientation.z();
  ret.w = orientation.w();
  return ret;
}

geometry_msgs::msg::Vector3 Qrientation2RPY(
  const double _x,
  const double _y,
  const double _z,
  const double _w)
{
  geometry_msgs::msg::Vector3 ret;
  tf2::Quaternion orientation(_x, _y, _z, _w);
  tf2::Matrix3x3 matrix(orientation);
  matrix.getRPY(ret.x, ret.y, ret.z);
  return ret;
}

std::string GetTime(int nowModo)
{  // {0:Ms1970, 1:_Y_M_D_H_M_S, 2:STANDARD}
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
    strftime(tmp, sizeof(tmp), "%Y.%m.%d-%H:%M:%S-%z", localtime(&timep));
    mapBackupsTime = tmp;
  } else {  // TimeMode::DETAILED
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y.%m.%d-%H:%M:%S", localtime(&timep));
    mapBackupsTime = tmp;

    timeval tv;
    gettimeofday(&tv, 0);
    int64_t _mapBackupsTime = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;
    std::string us = "";
    std::stringstream time_u_sec;
    time_u_sec << "-" << _mapBackupsTime;
    time_u_sec >> us;
    time_u_sec.clear();
    time_u_sec.str("");
    mapBackupsTime += us;
  }
  return mapBackupsTime;
}

double Angle2Radian(const double _degree)
{
  double degree = fmod(_degree, static_cast<double>(360.0));
  return static_cast<double>(degree * M_PI / static_cast<double>(180.0));
}

double Radian2Angle(const double _rad)
{
  double rad = fmod(_rad, static_cast<double>(M_PI * 2.0));
  return static_cast<double>(rad * static_cast<double>(180.0) / M_PI);
}

uint64_t GetTimeNs()
{
  struct timespec now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &now);
  return uint64_t(now.tv_sec * 1000000000 + now.tv_nsec);
}

bool JudgeToml(const std::string & _toml_file)
{
  if (access(_toml_file.c_str(), F_OK)) {
    ERROR(
      "Config file does not exist, config file dir:\n%s",
      _toml_file.c_str());
    return false;
  }
  if (access(_toml_file.c_str(), R_OK)) {
    ERROR(
      "Config file does not have read permissions, config file dir:\n%s",
      _toml_file.c_str());
    return false;
  }
  if (access(_toml_file.c_str(), W_OK)) {
    ERROR(
      "Config file does not have write permissions, config file dir:\n%s",
      _toml_file.c_str());
    return false;
  }
  return true;
}

bool GetWorkspace(std::string & _workspace)
{
  try {
    std::string pkg_dir = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    std::string config_file = pkg_dir + "/config/abilityset.toml";
    if (!JudgeToml(config_file)) {
      return false;
    }
    toml::value now_toml;
    if (!cyberdog::common::CyberdogToml::ParseFile(
        config_file.c_str(), now_toml))
    {
      ERROR("Toml config file is not in toml format, config file dir:\n%s", config_file.c_str());
      return false;
    }
    std::string workspace = toml::find<std::string>(
      now_toml, "vp", "init", "environment", "workspace");
    if ((workspace.size() > 1) &&
      (workspace.back() == '/'))
    {
      workspace.pop_back();
    }
    _workspace = (workspace.empty() ? ament_index_cpp::get_package_share_directory(
        "cyberdog_vp") : workspace) + "/workspace";
  } catch (const std::exception & e) {
    ERROR("Init data failed: %s.", e.what());
    return false;
  }
  return true;
}

bool Timeout(const uint64_t & _old_ns, uint64_t _timeout_ms)
{
  return ((GetTimeNs() - _old_ns) > (_timeout_ms * 1000000)) ? true : false;
}

std::string int2binary(const int _int)
{
  char bin[32] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
  char str[32] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
  int j = _int, i = 0;
  do {
    bin[i++] = (j % 2) ? '1' : '0';
  } while ((j /= 2) > 0);
  while (i-- > 0) {str[j++] = bin[i];}
  return std::string(str);
}

bool endsWith(const std::string & _str, const std::string & _suffix)
{
  return (_str.length() >= _suffix.length()) &&
         (_str.substr(_str.length() - _suffix.length()) == _suffix);
}

std::string covariance36(
  const std::array<double, 36> & _ary,
  const std::string _head,
  const int line_size)
{
  std::ostringstream ret;
  ret << "[";
  for (int i = 0; i < 35; i++) {
    if ((i > 0) && ((i % line_size) == 0)) {
      ret << _head;
    }
    ret << std::left << std::setw(8) << _ary[i] << ", ";
  }
  ret << std::left << std::setw(8) << _ary[35] << "]";
  return ret.str();
}

std::string covariance9(
  const std::array<double, 9> & _ary,
  const std::string _head,
  const int line_size)
{
  std::ostringstream ret;
  ret << "[";
  for (int i = 0; i < 8; i++) {
    if ((i > 0) && ((i % line_size) == 0)) {
      ret << _head;
    }
    ret << std::left << std::setw(8) << _ary[i] << ", ";
  }
  ret << std::left << std::setw(8) << _ary[8] << "]";
  return ret.str();
}

std::string sequenceGaitVector(
  const std::vector<MsgMotionSequenceGait> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - right_forefoot: = %s"
        "\n%s  - left_forefoot: = %s"
        "\n%s  - right_hindfoot: = %s"
        "\n%s  - left_hindfoot: = %s"
        "\n%s  - duration: = %ld",
        _spacing.c_str(), i,
        _spacing.c_str(), std::string(_vct.at(i).right_forefoot ? "True" : "False").c_str(),
        _spacing.c_str(), std::string(_vct.at(i).left_forefoot ? "True" : "False").c_str(),
        _spacing.c_str(), std::string(_vct.at(i).right_hindfoot ? "True" : "False").c_str(),
        _spacing.c_str(), std::string(_vct.at(i).left_hindfoot ? "True" : "False").c_str(),
        _spacing.c_str(), _vct.at(i).duration));
  }
  return ret.str();
}

std::string sequencePaceVector(
  const std::vector<MsgMotionSequencePace> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - twist:"
        "\n%s    - linear:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s    - angular:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s  - centroid:"
        "\n%s    - position:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s    - orientation:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s      - w = %lf"
        "\n%s  - weight:"
        "\n%s    - linear:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s    - angular:"
        "\n%s      - x = %lf"
        "\n%s      - y = %lf"
        "\n%s      - z = %lf"
        "\n%s  - right_forefoot:"
        "\n%s    - x = %lf"
        "\n%s    - y = %lf"
        "\n%s    - z = %lf"
        "\n%s    - w = %lf"
        "\n%s  - left_forefoot:"
        "\n%s    - x = %lf"
        "\n%s    - y = %lf"
        "\n%s    - z = %lf"
        "\n%s    - w = %lf"
        "\n%s  - right_hindfoot:"
        "\n%s    - x = %lf"
        "\n%s    - y = %lf"
        "\n%s    - z = %lf"
        "\n%s    - w = %lf"
        "\n%s  - left_hindfoot:"
        "\n%s    - x = %lf"
        "\n%s    - y = %lf"
        "\n%s    - z = %lf"
        "\n%s    - w = %lf"
        "\n%s  - friction_coefficient = %lf"
        "\n%s  - landing_gain = %lf"
        "\n%s  - use_mpc_track = %s"
        "\n%s  - duration_ms = %ld",
        _spacing.c_str(), i,
        _spacing.c_str(),
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).twist.linear.x,
        _spacing.c_str(), _vct.at(i).twist.linear.y,
        _spacing.c_str(), _vct.at(i).twist.linear.z,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).twist.angular.x,
        _spacing.c_str(), _vct.at(i).twist.angular.y,
        _spacing.c_str(), _vct.at(i).twist.angular.z,
        _spacing.c_str(),
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).centroid.position.x,
        _spacing.c_str(), _vct.at(i).centroid.position.y,
        _spacing.c_str(), _vct.at(i).centroid.position.z,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).centroid.orientation.x,
        _spacing.c_str(), _vct.at(i).centroid.orientation.y,
        _spacing.c_str(), _vct.at(i).centroid.orientation.z,
        _spacing.c_str(), _vct.at(i).centroid.orientation.w,
        _spacing.c_str(),
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).weight.linear.x,
        _spacing.c_str(), _vct.at(i).weight.linear.y,
        _spacing.c_str(), _vct.at(i).weight.linear.z,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).weight.angular.x,
        _spacing.c_str(), _vct.at(i).weight.angular.y,
        _spacing.c_str(), _vct.at(i).weight.angular.z,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).right_forefoot.x,
        _spacing.c_str(), _vct.at(i).right_forefoot.y,
        _spacing.c_str(), _vct.at(i).right_forefoot.z,
        _spacing.c_str(), _vct.at(i).right_forefoot.w,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).left_forefoot.x,
        _spacing.c_str(), _vct.at(i).left_forefoot.y,
        _spacing.c_str(), _vct.at(i).left_forefoot.z,
        _spacing.c_str(), _vct.at(i).left_forefoot.w,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).right_hindfoot.x,
        _spacing.c_str(), _vct.at(i).right_hindfoot.y,
        _spacing.c_str(), _vct.at(i).right_hindfoot.z,
        _spacing.c_str(), _vct.at(i).right_hindfoot.w,
        _spacing.c_str(),
        _spacing.c_str(), _vct.at(i).left_hindfoot.x,
        _spacing.c_str(), _vct.at(i).left_hindfoot.y,
        _spacing.c_str(), _vct.at(i).left_hindfoot.z,
        _spacing.c_str(), _vct.at(i).left_hindfoot.w,
        _spacing.c_str(), _vct.at(i).friction_coefficient,
        _spacing.c_str(), _vct.at(i).landing_gain,
        _spacing.c_str(), std::string(_vct.at(i).use_mpc_track ? "True" : "False").c_str(),
        _spacing.c_str(), _vct.at(i).duration));
  }
  return ret.str();
}

std::string msgFaceResVector(
  const std::vector<MsgFaceRes> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - result: = %d"
        "\n%s  - username: = %s"
        "\n%s  - age: = %lf"
        "\n%s  - emotion: = %lf",
        _spacing.c_str(), i,
        _spacing.c_str(), _vct.at(i).result,
        _spacing.c_str(), _vct.at(i).username.c_str(),
        _spacing.c_str(), _vct.at(i).age,
        _spacing.c_str(), _vct.at(i).emotion));
  }
  return ret.str();
}

std::string msgPersonnelVector(
  const std::vector<MsgPersonnel> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - id: = %d"
        "\n%s  - username: = %s"
        "\n%s  - voicestatus: = %d"
        "\n%s  - facestatus: = %d",
        _spacing.c_str(), i,
        _spacing.c_str(), _vct.at(i).id,
        _spacing.c_str(), _vct.at(i).username.c_str(),
        _spacing.c_str(), _vct.at(i).voicestatus,
        _spacing.c_str(), _vct.at(i).facestatus));
  }
  return ret.str();
}

std::string msgFaceResMap(
  const std::map<std::string, MsgFaceRes> & _map,
  const std::string _spacing)
{
  std::ostringstream ret;
  for (const auto & [key, value] : _map) {
    ret << std::string(
      FORMAT(
        "\n%s- [%s]"
        "\n%s  - result: = %d"
        "\n%s  - username: = %s"
        "\n%s  - age: = %lf"
        "\n%s  - emotion: = %lf",
        _spacing.c_str(), key.c_str(),
        _spacing.c_str(), value.result,
        _spacing.c_str(), value.username.c_str(),
        _spacing.c_str(), value.age,
        _spacing.c_str(), value.emotion));
  }
  return ret.str();
}

std::string msgPresetVector(
  const std::vector<MsgPreset> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - label_name: = %s"
        "\n%s  - physic_x: = %f"
        "\n%s  - physic_y: = %f"
        "\n%s  - physic_z: = 0.0",
        _spacing.c_str(), i,
        _spacing.c_str(), _vct.at(i).label_name.c_str(),
        _spacing.c_str(), _vct.at(i).physic_x,
        _spacing.c_str(), _vct.at(i).physic_y,
        _spacing.c_str()));
  }
  return ret.str();
}

std::string msgPresetMap(
  const std::map<std::string, MsgPreset> & _map,
  const std::string _spacing)
{
  std::ostringstream ret;
  for (const auto & [key, value] : _map) {
    ret << std::string(
      FORMAT(
        "\n%s- [%s]"
        "\n%s  - label_name: = %s"
        "\n%s  - physic_x: = %f"
        "\n%s  - physic_y: = %f"
        "\n%s  - physic_z: = 0.0",
        _spacing.c_str(), key.c_str(),
        _spacing.c_str(), value.label_name.c_str(),
        _spacing.c_str(), value.physic_x,
        _spacing.c_str(), value.physic_y,
        _spacing.c_str()));
  }
  return ret.str();
}

std::string msgTrainingWordsVector(
  const std::vector<MsgTrainingWords> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - trigger: = %s"
        "\n%s  - type: = %s"
        "\n%s  - value: = %s",
        _spacing.c_str(), i,
        _spacing.c_str(), _vct.at(i).trigger.c_str(),
        _spacing.c_str(), _vct.at(i).type.c_str(),
        _spacing.c_str(), _vct.at(i).value.c_str()));
  }
  return ret.str();
}

std::string msgTrainingWordsMap(
  const std::map<std::string, MsgTrainingWords> & _map,
  const std::string _spacing)
{
  std::ostringstream ret;
  for (const auto & [key, value] : _map) {
    ret << std::string(
      FORMAT(
        "\n%s- [%s]"
        "\n%s  - trigger: = %s"
        "\n%s  - type: = %s"
        "\n%s  - value: = %s",
        _spacing.c_str(), key.c_str(),
        _spacing.c_str(), value.trigger.c_str(),
        _spacing.c_str(), value.type.c_str(),
        _spacing.c_str(), value.value.c_str()));
  }
  return ret.str();
}

std::string msgDialogueResponseVector(
  const std::vector<DialogueResponse> & _vct,
  const std::string _spacing)
{
  std::ostringstream ret;
  size_t vct_size = _vct.size();
  for (size_t i = 0; i < vct_size; i++) {
    ret << std::string(
      FORMAT(
        "\n%s- [%ld]"
        "\n%s  - time_ns: = %ld"
        "\n%s  - data: = %s",
        _spacing.c_str(), i,
        _spacing.c_str(), _vct.at(i).time_ns,
        _spacing.c_str(), _vct.at(i).data.c_str()));
  }
  return ret.str();
}

std::string stringVector(const std::vector<std::string> & _vct)
{
  std::ostringstream ret;
  ret << "[";
  if (!_vct.empty()) {
    size_t for_max = _vct.size() - 1;
    for (size_t i = 0; i < for_max; i++) {
      ret << _vct.at(i) << ", ";
    }
    ret << _vct.at(for_max);
  }
  ret << "]";
  return ret.str();
}

std::string intVectorToString(const std::vector<int> & _vct)
{
  std::ostringstream ret;
  ret << "[";
  if (!_vct.empty()) {
    size_t for_max = _vct.size() - 1;
    for (size_t i = 0; i < for_max; i++) {
      ret << std::to_string(_vct.at(i)) << ", ";
    }
    ret << std::to_string(_vct.at(for_max));
  }
  ret << "]";
  return ret.str();
}
}   // namespace cyberdog_visual_programming_abilityset
