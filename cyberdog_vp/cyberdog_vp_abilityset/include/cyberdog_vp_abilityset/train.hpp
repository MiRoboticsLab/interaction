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

#ifndef CYBERDOG_VP_ABILITYSET__TRAIN_HPP_
#define CYBERDOG_VP_ABILITYSET__TRAIN_HPP_

#include <string>
#include <memory>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       train.hpp
    \brief      训练模块。
    \details    创建及初始化训练模块，以便训练接口在任务中正常使用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \note       训练词的增、删、改逻辑由开发者用户完成。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Train final : public Base
{
public:
  Train()
  : Base(std::string(__FUNCTION__)) {}
  ~Train() {}

  TrainingWordsRecognizedSeviceResponse GetTrainingWordsSet(
  );                                              /*!< 获取训练词集合 */
  TrainingWordsRecognizedMessageResponse TrainingWordsRecognized(
    const int timeout = -1);                      /*!< 识别到训练词 */

private:
  rclcpp::CallbackGroup::SharedPtr
    training_words_sub_cb_group_ {nullptr};       /*!< [回调组]训练词响应 */
  rclcpp::CallbackGroup::SharedPtr
    user_dialogue_sub_cb_group_ {nullptr};        /*!< [回调组]用户对话响应 */
  rclcpp::CallbackGroup::SharedPtr
    training_words_cli_cb_group_ {nullptr};       /*!< [回调组]训练词服务 */

  rclcpp::Subscription<MsgTrainingWords>::SharedPtr
    training_words_recognition_sub_ptr_
  {nullptr};                                      /*!< [监听器]训练词响应 */
  rclcpp::Subscription<MsgString>::SharedPtr
    user_dialogue_message_sub_ptr_
  {nullptr};                                      /*!< [监听器]用户对话响应 */
  rclcpp::Client<SrvTrainingWords>::SharedPtr
    training_words_recognition_cli_ptr_
  {nullptr};                                      /*!< [客户端]训练词服务 */

  MsgTrainingWords training_words_;               /*!< 训练词目标 */
  SrvTrainingWords::Response
    training_words_set_;                          /*!< 训练词集合 */
  bool training_words_update_ {false};            /*!< 训练词更新 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubTrainingWordsRecognitionCB(
    const MsgTrainingWords::SharedPtr);           /*!< 训练词响应数据回调 */
  void SubUserDialogueCB(
    const MsgString::SharedPtr);                  /*!< 用户对话数据回调 */
  State RequestTrainingWordsRecognizedSrv(
    SrvTrainingWords::Response &,
    std::shared_ptr<SrvTrainingWords::Request>,
    const int _service_start_timeout = 10);       /*!< 请求训练词服务 */
};  // class Train
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__TRAIN_HPP_
