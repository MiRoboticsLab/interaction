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

#ifndef CYBERDOG_VP_ABILITYSET__FACE_HPP_
#define CYBERDOG_VP_ABILITYSET__FACE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "cyberdog_vp_abilityset/common.hpp"
#include "cyberdog_vp_abilityset/base.hpp"

namespace cyberdog_visual_programming_abilityset
{
/*! \file       face.hpp
    \brief      人脸识别模块。
    \details    创建及初始化人脸识别模块，以便任务调用人脸识别功能。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化设备。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Face final : public Base
{
public:
  Face()
  : Base(std::string(__FUNCTION__)) {}
  ~Face() {}
  FaceRecognizedSeviceResponse Recognized(
    const std::vector<std::string> &,
    const bool _and_operation = false,
    const int _duration = -1);                    /*!< 识别到目标人员人脸 */
  FaceSeviceResponse CancelRecognize(
    const int _timeout = -1);                     /*!< 识别到目标人员人脸 */

private:
  std::map<std::string /*name*/, std::string /*id*/>
  face_target_id_;                                /*!< 人脸识别目标id集合 */
  std::vector<MsgFaceRes> face_recognition_data_; /*!< 人脸识别到的目标集合 */
  bool face_recognition_life_cycle_ {false};      /*!< 人脸识别生命周期内 */
  std::vector<std::string> voiceprint_target_;    /*!< 声纹识别目标 */

  rclcpp::CallbackGroup::SharedPtr
    face_sub_cb_group_ {nullptr};                 /*!< [回调组]人脸识别响应 */
  rclcpp::CallbackGroup::SharedPtr
    face_cli_cb_group_ {nullptr};                 /*!< [回调组]人脸识别服务 */

  rclcpp::Subscription<MsgFaceRes>::SharedPtr
    face_recognition_sub_ptr_ {nullptr};          /*!< [监听器]人脸识别响应 */
  rclcpp::Client<SrvFaceRec>::SharedPtr
    face_recognition_cli_ptr_ {nullptr};          /*!< [客户端]人脸识别服务 */

private:
  bool SetData(const toml::value &);              /*!< 设置数据 */
  bool SetMechanism(const toml::value &);         /*!< 设置机制 */
  void SubFaceRecognitionCB(
    const MsgFaceRes::SharedPtr);                 /*!< 人脸识别响应数据回调 */
  std::shared_ptr<SrvFaceRec::Request>
  GetFaceRecognitionRequest();                    /*!< 获取人脸识别服务请求 */
  bool RequestFaceRecognizedSrv(
    SrvFaceRec::Response &,
    std::shared_ptr<SrvFaceRec::Request>,
    const int _service_start_timeout = 10);       /*!< 请求人脸识别服务 */
};  // class Face
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__FACE_HPP_
