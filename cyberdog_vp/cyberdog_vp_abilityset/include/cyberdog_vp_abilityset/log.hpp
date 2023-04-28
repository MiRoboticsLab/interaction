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

#ifndef CYBERDOG_VP_ABILITYSET__LOG_HPP_
#define CYBERDOG_VP_ABILITYSET__LOG_HPP_

#include <string>
#include "cyberdog_vp_abilityset/common.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       log.hpp
    \brief      可视化编程SDK的日志模块。
    \details    创建及初始化SDK模块的日志逻辑。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Log
{
public:
  /*! 日志级别 */
  enum Rank
  {
    debug = 0,                                    /*!< 调试 */
    info,                                         /*!< 信息 */
    warn,                                         /*!< 警告 */
    error,                                        /*!< 错误 */
    fatal,                                        /*!< 致命 */
  };
  Log(Log & other) = delete;
  void operator=(const Log &) = delete;
  static Log * GetInstance(
    rclcpp::Node::SharedPtr _node_ptr,
    const std::string & _task_id,
    const toml::value & _params_toml);            /*!< 获取接口 */

  bool Ok() {return this->is_ok_;}                /*!< 日志就绪 */
  void On() {this->log_ = true;}                  /*!< 打开日志 */
  void Off() {this->log_ = false;}                /*!< 关闭日志 */

  template<typename ... Args>
  void Record(
    Rank rank,
    bool condition,
    std::string role,
    const char * format,
    const Args & ... args)                         /*!< 记录日志 */
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!this->log_ || !condition) {
      return;
    }
    char * message = nullptr;
    std::string rank_str = "";
    auto update_message = [&]() -> bool {
        int size = std::snprintf(nullptr, 0, format, args ...);
        if (size > 0) {
          size++;
          message = new(std::nothrow) char[size];
          if (message != nullptr) {
            std::snprintf(message, size, format, args ...);
            message = this->Trim(message);
          }
        }
        return message != nullptr;
      };
    auto record_terminal = [&]() {
        switch (rank) {
          case Rank::debug:
            DEBUG("[%s] %s", role.c_str(), message);
            rank_str = "debug";
            break;
          case Rank::info:
            INFO("[%s] %s", role.c_str(), message);
            rank_str = "info";
            break;
          case Rank::warn:
            WARN("[%s] %s", role.c_str(), message);
            rank_str = "warn";
            break;
          case Rank::error:
            ERROR("[%s] %s", role.c_str(), message);
            rank_str = "error";
            break;
          case Rank::fatal:
            FATAL("[%s] %s", role.c_str(), message);
            rank_str = "fatal";
            break;
          default:
            std::cout << "[" << role << "]" << message << std::endl;
            rank_str = "Other";
            break;
        }
      };
    auto record_file = [&]() {
        if (this->is_ok_ && this->record_) {
          this->log_file_ << rank_str << "," << GetTime(TimeMode::DETAILED) << "," << role <<
            ",\"" << message << "\"" << std::endl;
          this->log_file_.flush();
        }
      };
    auto record_topic = [&]() {
        if (this->is_ok_ && this->publish_log_) {
          MsgHeader msg;
          msg.stamp = this->node_ptr_->now();
          msg.frame_id = message;
          this->publish_log_pub_->publish(msg);
        }
      };
    if (update_message()) {
      record_terminal();
      record_file();
      record_topic();
      delete[] message;
      message = nullptr;
    }
  }

private:
  static Log * pinstance_ptr;                     /*!< 实例对象 */
  static std::mutex mutex_;                       /*!< 互斥量 */
  std::string task_id_ {""};                      /*!< 任务id */
  std::string file_ {""};                         /*!< 记录文件 */
  std::ofstream log_file_;                        /*!< 日志文件流 */
  bool log_ {true};                               /*!< 日志开关 */
  bool record_ {false};                           /*!< 文件记录开关 */
  bool publish_log_ {false};                      /*!< 发布消息开关 */
  bool is_ok_ {false};                            /*!< 是否就绪 */

  rclcpp::Node::SharedPtr node_ptr_ {nullptr};    /*!< 凡人节点 */

  rclcpp::CallbackGroup::SharedPtr
    publish_log_cb_group_ {nullptr};              /*!< [回调组]发布日志 */
  rclcpp::Publisher<MsgHeader>::SharedPtr
    publish_log_pub_ {nullptr};                   /*!< [发布器]发布日志 */

protected:
  Log(
    rclcpp::Node::SharedPtr _node_ptr,
    const std::string _task_id,
    const toml::value & _params_toml)
  : task_id_(_task_id),
    node_ptr_(_node_ptr)
  {
    this->is_ok_ = this->SetData(_params_toml) && this->SetMechanism(_params_toml);
  }
  ~Log()
  {
    if (this->log_file_.is_open()) {
      std::lock_guard<std::mutex> lock(mutex_);
      this->log_file_.close();
    }
  }
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  char * Trim(char * str);                        /*!< 裁剪字符串收尾空白字符 */
};  // class Log
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__LOG_HPP_
