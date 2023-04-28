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

#include "cyberdog_vp_engine/common.hpp"

namespace cyberdog_visual_programming_engine
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

bool Shell(const std::string & _command, int & _code, std::string & _message)
{
  try {
    INFO("Shell: %s", _command.c_str());
    _message = "";
    if (!_command.empty()) {
      std::string _code_cmd = _command + "; echo $?";  //  | xargs echo
      std::string _code_str = "";
      FILE * fstream_ptr = nullptr;
      fstream_ptr = popen(_code_cmd.c_str(), "r");
      if (fstream_ptr != nullptr) {
        char buffer[LINE_MAX_SIZE];
        while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
          _code_str = buffer;
          memset(buffer, '\0', sizeof(buffer));
          _message += _code_str;
        }
        pclose(fstream_ptr);
        fstream_ptr = nullptr;
        _code = std::atoi(_code_str.c_str());
        if (_code == static_cast<int>(0)) {
          return true;
        } else {
          _code = static_cast<int>(ShellEnum::command_error);
          _code_cmd = _command + " 2> /dev/stdout";
          fstream_ptr = popen(_code_cmd.c_str(), "r");
          if (fstream_ptr != nullptr) {
            _code_str = "";
            while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
              _code_str += std::string(buffer);
              memset(buffer, '\0', sizeof(buffer));
            }
            pclose(fstream_ptr);
            fstream_ptr = nullptr;
          }
          _message = "Shell command is error.\n - command : " + _command +
            "\n - code : " + _message + " - error : " + _code_str;
        }
      } else {
        _code = static_cast<int>(ShellEnum::command_popen);
        _message = "Canot shell command popen.\n - command : " + _command +
          "\n - error : " + strerror(errno);
      }
    } else {
      _code = static_cast<int>(ShellEnum::command);
      _message = "Shell command is empty.\n - command : " + _command;
    }
  } catch (const std::exception & e) {
    _code = static_cast<int>(ShellEnum::shell);
    _message = "Shell command is error.\n - command : " + _command +
      "\n - error : " + e.what();
  }
  ERROR("Shell: %s, %s", _command.c_str(), _message.c_str());
  return false;
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

bool JudgeConfileFile(std::string _file)
{
  INFO("Judge params config file dir:%s.", _file.c_str());
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
  toml::value now_toml;
  if (!cyberdog::common::CyberdogToml::ParseFile(
      _file.c_str(), now_toml))
  {
    ERROR("Toml config file is not in toml format, config file dir:\n%s", _file.c_str());
    return false;
  }
  return true;
}

bool GetWorkspace(std::string & _workspace)
{
  try {
    std::string pkg_dir = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    std::string config_file = pkg_dir + "/config/abilityset.toml";
    if (!JudgeConfileFile(config_file)) {
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

bool Mkdir(std::string & _tar_path)
{
  // return std::filesystem::create_directories(_tar_path);
  if (access(_tar_path.c_str(), F_OK) == 0) {
    return true;
  }
  if (mkdir(_tar_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
    ERROR("Mkdir <%s> failed: %s.", _tar_path.c_str(), std::strerror(errno));
    return false;
  }
  return true;
  // 函数原型: int mkdir(const char *pathname, mode_t mode);
  // 函数说明: mkdir()函数以mode方式创建一个以参数pathname命名的目录, mode定义新创建目录的权限。
  // 返回值: 若目录创建成功, 则返回0; 否则返回-1, 并将错误记录到全局变量errno中。
  // mode方式:
  //     S_IRWXU 00700权限, 代表该文件所有者拥有读, 写和执行操作的权限
  //     S_IRUSR(S_IREAD) 00400权限, 代表该文件所有者拥有可读的权限
  //     S_IWUSR(S_IWRITE) 00200权限, 代表该文件所有者拥有可写的权限
  //     S_IXUSR(S_IEXEC) 00100权限, 代表该文件所有者拥有执行的权限
  //     S_IRWXG 00070权限, 代表该文件用户组拥有读, 写和执行操作的权限
  //     S_IRGRP 00040权限, 代表该文件用户组拥有可读的权限
  //     S_IWGRP 00020权限, 代表该文件用户组拥有可写的权限
  //     S_IXGRP 00010权限, 代表该文件用户组拥有执行的权限
  //     S_IRWXO 00007权限, 代表其他用户拥有读, 写和执行操作的权限
  //     S_IROTH 00004权限, 代表其他用户拥有可读的权限
  //     S_IWOTH 00002权限, 代表其他用户拥有可写的权限
  //     S_IXOTH 00001权限, 代表其他用户拥有执行的权限
}

std::string Subreplace(
  const std::string & resource_str,
  const std::string & sub_str,
  const std::string & new_str)
{
  std::string dst_str = resource_str;
  std::string::size_type pos = 0;
  std::string::size_type add_pos = new_str.size();
  while ((pos = dst_str.find(sub_str, (pos ? (pos + add_pos) : pos))) != std::string::npos) {
    dst_str.replace(pos, sub_str.length(), new_str);
  }
  return dst_str;
}
}   // namespace cyberdog_visual_programming_engine
