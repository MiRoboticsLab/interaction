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

#ifndef CYBERDOG_VP_ABILITYSET__BASE_HPP_
#define CYBERDOG_VP_ABILITYSET__BASE_HPP_

#include <string>
#include <memory>
#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/log.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       base.hpp
    \brief      可视化编程SDK的基础模块。
    \details    创建及初始化SDK模块的通用逻辑。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Base
{
public:
  explicit Base(std::string _logger)
  : logger_(_logger) {}
  Base(std::string _logger, bool _heartbeat)
  : logger_(_logger), heartbeat_(_heartbeat) {}
  virtual ~Base() {}
  State state_;                                   /*!< 整体状态 */
  std::shared_ptr<State> transient_state_ptr_;    /*!< 瞬时状态 */

protected:
  bool log_ {true};                               /*!< 日志开关 */
  Log * log_ptr_ {nullptr};                       /*!< 日志 */
  std::string logger_ {""};                       /*!< 日志名称 */
  uint16_t timeout_wait_for_service_ {60};        /*!< 等待服务启动超时 */
  uint16_t timeout_wait_for_ {10};                /*!< 等待服务响应超时 */
  uint64_t timens_ {0};                           /*!< 数据时间 */
  std::string task_id_ {""};                      /*!< 任务id */
  rclcpp::Node::SharedPtr
    node_immortal_ptr_ {nullptr};                 /*!< 神仙节点 */
  rclcpp::Node::SharedPtr
    node_mortal_ptr_ {nullptr};                   /*!< 凡人节点 */
  bool heartbeat_ {false};                        /*!< 是否开启心跳 */

  rclcpp::CallbackGroup::SharedPtr
    heartbeat_cb_group_ {nullptr};                /*!< [回调组]心跳 */
  rclcpp::TimerBase::SharedPtr
    heartbeat_ptr_ {nullptr};                     /*!< [定时器]心跳 */

protected:
  void SetState(const StateCode);                 /*!< 设置状态 */
  State GetState(
    const std::string,
    const StateCode);                             /*!< 获取状态 */
  std::string GetDescribe(
    const std::string,
    const StateCode);                             /*!< 获取状态描述 */
  void Heartbeat();                               /*!< 心跳 */

public:
  bool Init(
    const std::string,
    const rclcpp::Node::SharedPtr,
    const rclcpp::Node::SharedPtr,
    const std::shared_ptr<State>,
    const toml::value &);                         /*!< 初始化 */
  bool GetToml(
    const std::string &,
    toml::value &);                               /*!< 获取 toml */
  virtual bool SetData(const toml::value &)
  {return true;}                                  /*!< 设置数据 */
  virtual bool SetMechanism(const toml::value &)
  {return true;}                                  /*!< 设置机制 */
  virtual void SetLog(const bool);                /*!< 设置日志 */

  template<typename ... Args>
  void DebugIf(bool condition, const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::debug, condition, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void InfoIf(bool condition, const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::info, condition, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void WarnIf(bool condition, const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::warn, condition, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void ErrorIf(bool condition, const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::error, condition, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void FatalIf(bool condition, const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::fatal, condition, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void Debug(const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::debug, true, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void Info(const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::info, true, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void Warn(const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::warn, true, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void Error(const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::error, true, this->logger_, format, args ...);
    }
  }

  template<typename ... Args>
  void Fatal(const char * format, const Args & ... args)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::fatal, true, this->logger_, format, args ...);
    }
  }

  void DebugIf(bool condition, const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::debug, condition, this->logger_, "%s", message);
    }
  }

  void InfoIf(bool condition, const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::info, condition, this->logger_, "%s", message);
    }
  }

  void WarnIf(bool condition, const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::warn, condition, this->logger_, "%s", message);
    }
  }

  void ErrorIf(bool condition, const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::error, condition, this->logger_, "%s", message);
    }
  }

  void FatalIf(bool condition, const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::fatal, condition, this->logger_, "%s", message);
    }
  }

  void Debug(const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::debug, true, this->logger_, "%s", message);
    }
  }

  void Info(const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::info, true, this->logger_, "%s", message);
    }
  }

  void Warn(const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::warn, true, this->logger_, "%s", message);
    }
  }

  void Error(const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::error, true, this->logger_, "%s", message);
    }
  }

  void Fatal(const char * message)
  {
    if (this->log_) {
      this->log_ptr_->Record(Log::Rank::fatal, true, this->logger_, "%s", message);
    }
  }
};  // class Base
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__BASE_HPP_
