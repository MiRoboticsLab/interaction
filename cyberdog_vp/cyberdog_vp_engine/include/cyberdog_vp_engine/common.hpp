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
#ifndef CYBERDOG_VP_ENGINE__COMMON_HPP_
#define CYBERDOG_VP_ENGINE__COMMON_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <cyberdog_common/cyberdog_json.hpp>
#include <cyberdog_machine/cyberdog_fs_machine.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <protocol/msg/train_plan.hpp>
#include <protocol/msg/label.hpp>
#include <protocol/msg/user_information.hpp>
#include <protocol/msg/audio_play_extend.hpp>
#include <std_msgs/msg/string.hpp>
#include <protocol/msg/visual_programming_operate.hpp>

#include <protocol/srv/visual_programming_operate.hpp>
#include <protocol/srv/get_map_label.hpp>
#include <protocol/srv/bes_http.hpp>
#include <protocol/srv/all_user_search.hpp>
#include <protocol/srv/train_plan_all.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <mutex>
#include <tuple>
#include <regex>
#include <algorithm>

namespace cyberdog_visual_programming_engine
{
/*! \file       common.hpp
    \brief      可视化编程引擎的共享模块。
    \details    功能繁杂，详情参见具体内容。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        依据参数列表均可直接调用。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
/*! 约束调用 shell 接口时的异常返回值的合法值 */
enum ShellEnum_
{
  shell = 1993,                                   /*!< 异常结束, 执行shell命令错误 */
  command,                                        /*!< 异常结束, 待执行命令错误 */
  command_popen,                                  /*!< 异常结束, 无法执行命令 */
  command_error,                                  /*!< 异常结束, 执行命令失败 */
};
/*! 约束获取时间接口入参的合法值 */
enum TimeMode_
{
  Ms1970 = 0,                                     /*!< 1970年1月1日到现在的时间(毫秒) */
  _Y_M_D_H_M_S,                                   /*!< "*Y*M*D*H*M*S" */
  STANDARD,                                       /*!< "*Y.*M.*D-*H:*M:*S" */
};

/*! 通用状态约束 */
enum StateEnum
{
  normally = 0,                                   /*!< 正常结束 */
  abnormally_mkdir_path = 21,                     /*!< 21:异常结束, 无法创建路径 */
  abnormally_open_file,                           /*!< 22:异常结束, 无法打开文件 */
  abnormally_build,                               /*!< 23:异常结束, 无法构建 */
  abnormally_register,                            /*!< 24:异常结束, 无法注册 */
  abnormally_update_list,                         /*!< 25:异常结束, 无法更新列表 */
  abnormally_perform,                             /*!< 26:异常结束, 无法执行 */
  abnormally_operate,                             /*!< 27:异常结束, 当前操作非法 */
  abnormally_request,                             /*!< 28:异常结束, 请求错误 */
  abnormally_other_errors,                        /*!< 29:异常结束, 其他错误 */
  abnormally_decorate_body,                       /*!< 30:异常结束, 装饰身体失败 */
  service_request_interrupted,                    /*!< 31:异常结束, 服务被打断 */
  service_appear_timeout,                         /*!< 32:异常结束, 等待服务上线超时 */
  service_request_timeout,                        /*!< 33:异常结束, 请求服务超时 */
};

namespace py = pybind11;
using CyberdogJson =
  cyberdog::common::CyberdogJson;                 /*!< json 解析类型 */

using TrainingWordsMsg =
  protocol::msg::TrainPlan;                       /*!< 训练词消息 */
using AudioPlayExtendMsg =
  protocol::msg::AudioPlayExtend;                 /*!< 语音消息:在线 */
using PersonnelMsg =
  protocol::msg::UserInformation;                 /*!< 人员信息 */
using PresetMsg = protocol::msg::Label;           /*!< [topic 类型]预置点 消息 */
using GRPCMsg = std_msgs::msg::String;            /*!< [topic 类型]GRPC 消息 */
using ASRMsg = protocol::msg::TrainPlan;          /*!< [topic 类型]自动语音识别 消息 */
using OperateMsg =
  protocol::msg::VisualProgrammingOperate;        /*!< [topic 类型]任务操作 */

using OperateSrv =
  protocol::srv::VisualProgrammingOperate;        /*!< [service 类型]任务操作 */
using PersonnelSrv =
  protocol::srv::AllUserSearch;                   /*!< [service 类型]人员底库 */
using TrainingWordsSrv =
  protocol::srv::TrainPlanAll;                    /*!< [service 类型]训练词集合 */
using PresetSrv = protocol::srv::GetMapLabel;     /*!< [service 类型]预置点 */
using HTPSrv = protocol::srv::BesHttp;            /*!< [service 类型]后端通信 */

using ShellEnum = ShellEnum_;                     /*!< shell 接口返回值合法类型 */
using TimeMode = TimeMode_;                       /*!< 时间获取接口入参类型 */

static const rclcpp::QoS SubscriptionQos =
  rclcpp::ParametersQoS();                        /*!< [质量服务]监听器 */
static const rclcpp::QoS PublisherQos =
  rclcpp::ParametersQoS();                        /*!< [质量服务]发布器 */
static const rclcpp::QoS ClientQos =
  rclcpp::SystemDefaultsQoS();                    /*!< [质量服务]客户端 */
static const rclcpp::QoS ServicesQos =
  rclcpp::ServicesQoS();                          /*!< [质量服务]服务端 */

static const char PYTHON_PREFIX[4] {"mi_"};       /*!< python 源文件名称头部 */
static const char PYTHON_SRC[4] {"src"};          /*!< python 源文件路径 */
static const unsigned int LINE_MAX_SIZE {16384};  /*!< 行最大值:2kb */
static std::mutex registry_toml_mutex;            /*!< [互斥锁] 注册文件 */
static std::mutex registry_related_toml_mutex;    /*!< [互斥锁] 注册相关文件 */
static const std::vector<std::string> LICENSES = {
  "# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.",
  "#",
  "# Licensed under the Apache License, Version 2.0 (the \"License\");",
  "# you may not use this file except in compliance with the License.",
  "# You may obtain a copy of the License at",
  "#",
  "#     http://www.apache.org/licenses/LICENSE-2.0",
  "#",
  "# Unless required by applicable law or agreed to in writing, software",
  "# distributed under the License is distributed on an \"AS IS\" BASIS,",
  "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.",
  "# See the License for the specific language governing permissions and",
  "# limitations under the License."
};                                                /*!< 文件许可证 */

/*! 参数约束:后端消息 */
struct BackendMsg
{
  std::string code;                               /*!< 状态码 */
  std::string message;                            /*!< 消息 */
  std::string request_id;                         /*!< 请求id */
  std::vector<std::string> data;                  /*!< 数据 */
};
/*! {0:Ms1970, 1:_Y_M_D_H_M_S, 2:STANDARD} */
std::string GetTime(
  int nowModo = 0);                               /*!< 获取时间戳 */
bool Shell(
  const std::string &,
  int &,
  std::string &);                                 /*!< 执行shell */
std::vector<std::string> GetVector(
  const std::string &, char,
  const std::string & _head = "");                /*!< 获取向量 */
bool JudgeConfileFile(std::string _file);         /*!< 判断配置文件权限 */
bool GetWorkspace(std::string &);                 /*!< 获取工作空间 */
bool Mkdir(std::string &);                        /*!< 创建文件夹 */
std::string Subreplace(
  const std::string &,
  const std::string &,
  const std::string &);                           /*!< 对字符串中所有指定的子串进行替换 */
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__COMMON_HPP_
