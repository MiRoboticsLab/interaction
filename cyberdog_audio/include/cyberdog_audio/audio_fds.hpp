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
#ifndef CYBERDOG_AUDIO__AUDIO_FDS_HPP_
#define CYBERDOG_AUDIO__AUDIO_FDS_HPP_

#include <memory>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_fds.hpp"
#include "cyberdog_common/cyberdog_model.hpp"
#include "cyberdog_audio/audio_play.hpp"


namespace cyberdog
{
namespace interaction
{
enum ShellEnum
{
  shell = 1993,     // 异常结束, 执行shell命令错误
  command,          // 异常结束, 待执行命令错误
  command_popen,    // 异常结束, 无法执行命令
  command_error,    // 异常结束, 执行命令失败
};
class AudioFds
{
public:
  AudioFds()
  {
    t_fds_ = std::thread(
      [this]() {
        while (!exit_ && rclcpp::ok()) {
          std::unique_lock<std::mutex> lck(fds_mtx_);
          update_cv_.wait(
            lck, [this]() -> bool {
              return (is_update_ == true) || (!rclcpp::ok());
            });
          if (exit_) {
            break;
          }
          try {
            Init();
            audio_play_ptr_->LoadSoundYaml();
          } catch (const std::exception & e) {
            std::cerr << e.what() << '\n';
            INFO("fds thread exception:%s", e.what());
          } catch (...) {
            INFO("fds thread unkown exception.");
          }
          is_update_ = false;
        }
      });
  }

  ~AudioFds()
  {
    std::unique_lock<std::mutex> lck(fds_mtx_);
    exit_ = true;
    is_update_ = true;
    update_cv_.notify_all();
    if (t_fds_.joinable()) {
      t_fds_.join();
    }
  }
  void get_audio_play_ptr(std::shared_ptr<AudioPlay> ptr)
  {
    audio_play_ptr_ = ptr;
  }
  void Update()
  {
    std::unique_lock<std::mutex> lck(fds_mtx_);
    is_update_ = true;
    update_cv_.notify_all();
  }

private:
  bool Init()
  {
    std::string file_path = SDCARD + AUDIO_CATEGORY + "/" + BASIS_MODULE;
    std::string version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + CAMERA_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + FACE_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + MUSIC_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + TRACKING_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + WIFI_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + YAML_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + POWER_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    file_path = SDCARD + AUDIO_CATEGORY + "/" + AUDIO_MODULE;
    version_file = file_path + "/" + VERSION_FILE;
    if (!File_Exist(file_path)) {
      if (!Mk_Folder(file_path)) {
        return false;
      }
    }
    if (!File_Exist(version_file)) {
      if (!Mk_File(version_file)) {
        return false;
      }
    }
    if (!Download(BASIS_MODULE)) {
      return false;
    }
    if (!Download(CAMERA_MODULE)) {
      return false;
    }
    if (!Download(FACE_MODULE)) {
      return false;
    }
    if (!Download(MUSIC_MODULE)) {
      return false;
    }
    if (!Download(TRACKING_MODULE)) {
      return false;
    }
    if (!Download(WIFI_MODULE)) {
      return false;
    }
    if (!Download(YAML_MODULE)) {
      return false;
    }
    if (!Download(POWER_MODULE)) {
      return false;
    }
    if (!Download(AUDIO_MODULE)) {
      return false;
    }
    return true;
  }

  bool Download(std::string module)
  {
    cyberdog::common::cyberdog_model model(module, false, "", SDCARD, AUDIO_CATEGORY);
    model.SetTimeout(300);
    int32_t code = model.UpdateModels();
    if (code == 0) {
      INFO("download model from Fds successfully");
      if (model.Load_Model_Check()) {
        model.Post_Process();
        INFO("update model from Fds successfully");
      }
    } else {
      if (model.Load_Model_Check()) {
        model.Post_Process();
        INFO("left unload models, update local model successfully");
      }
    }
    return true;
  }

  bool File_Exist(const std::string path)
  {
    std::string filepath = path;
    if (access(filepath.c_str(), F_OK) != 0) {
      INFO("%s do not exist!", filepath.c_str());
      return false;
    } else {
      INFO("%s exist!", filepath.c_str());
      return true;
    }
  }

  bool Mk_Folder(std::string & path)
  {
    if (access(path.c_str(), F_OK) != 0) {
      int shell_code;
      std::string shell_message;
      bool result = this->Shell("mkdir -p " + path, shell_code, shell_message);
      if (result != true) {
        INFO("mkdir failed");
        return false;
      } else {
        INFO("mkdir successfully");
        return true;
      }
    } else {
      INFO("%s exist!", path.c_str());
      return true;
    }
  }

  bool Mk_File(std::string & file)
  {
    int shell_code;
    std::string shell_message;
    std::string cmd = "echo version = \\\"0.0\\\" > " + file;
    bool result = this->Shell(cmd, shell_code, shell_message);
    if (result != true) {
      INFO("mkfile failed");
      return false;
    } else {
      INFO("mkfile successfully");
      return true;
    }
  }

  bool Shell(const std::string & _command, int & _code, std::string & _message)
  {
    try {
      DEBUG("Shell: %s", _command.c_str());
      _message = "";
      if (!_command.empty()) {
        std::string _code_cmd = _command + "; echo $?";
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

private:
  static const unsigned int LINE_MAX_SIZE {16384};
  const std::string SDCARD = "/SDCARD/";
  const std::string AUDIO_CATEGORY = "sound";
  const std::string VERSION_FILE = "version.toml";
  const std::string BASIS_MODULE = "basis";
  const std::string CAMERA_MODULE = "camera";
  const std::string FACE_MODULE = "face";
  const std::string MUSIC_MODULE = "music";
  const std::string TRACKING_MODULE = "tracking";
  const std::string WIFI_MODULE = "wifi";
  const std::string YAML_MODULE = "yaml";
  const std::string POWER_MODULE = "power";
  const std::string AUDIO_MODULE = "audio";
  std::thread t_fds_;
  std::mutex fds_mtx_;
  bool is_update_{false};
  bool exit_{false};
  std::condition_variable update_cv_;
  std::shared_ptr<AudioPlay> audio_play_ptr_;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__AUDIO_FDS_HPP_
