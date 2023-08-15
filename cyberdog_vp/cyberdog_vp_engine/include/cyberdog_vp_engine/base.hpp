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
#ifndef CYBERDOG_VP_ENGINE__BASE_HPP_
#define CYBERDOG_VP_ENGINE__BASE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "cyberdog_vp_engine/common.hpp"
#include "cyberdog_vp_engine/frontend_message.hpp"
#include "cyberdog_vp_engine/python_interpreter.hpp"

namespace cyberdog_visual_programming_engine
{
/*! \file       base.hpp
    \brief      可视化编程引擎的基础模块。
    \details    创建及初始化“任务编程”和“模块编程”模块的通用逻辑。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化任务编程或模块编程模块。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
class Base
{
public:
  explicit Base(std::string _type);
  ~Base();
  bool Init(
    const rclcpp::Node::SharedPtr &,
    const std::shared_ptr<PythonInterpreter> &,
    const toml::value &);                         /*!< 初始化 */
  virtual bool InitData();                        /*!< 初始化数据 */

private:
  const std::vector<std::string> BASH_CONFIGURATION = {
    "# !/bin/sh",
    "export ROS_VERSION=2",
    "export ROS_PYTHON_VERSION=3",
    "export ROS_LOCALHOST_ONLY=0",
    "export ROS_DISTRO=galactic",
    "export ROS_DOMAIN_ID=42",
    "export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/gazebo-11/plugins:/opt/ros2/galactic/opt/yaml_cpp_vendor/lib:/opt/ros2/galactic/opt/rviz_ogre_vendor/lib:/opt/ros2/galactic/lib:/opt/ros2/cyberdog/lib",  // NOLINT
    "export PYTHONPATH=/opt/ros2/cyberdog/lib/python3.6/site-packages:/opt/ros2/galactic/lib/python3.6/site-packages",  // NOLINT
    "export AMENT_PREFIX_PATH=/opt/ros2/cyberdog:/opt/ros2/galactic",
    "export CMAKE_PREFIX_PATH=/opt/ros2/cyberdog:/opt/ros2/galactic",
    "export COLCON_PREFIX_PATH=/opt/ros2/cyberdog:/opt/ros2/galactic",
    "export PATH=/opt/ros2/galactic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin",  // NOLINT
    "export PKG_CONFIG_PATH=/opt/ros2/galactic/lib/aarch64-linux-gnu/pkgconfig:/opt/ros2/galactic/lib/pkgconfig",  // NOLINT
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    "export CYCLONEDDS_URI=file:///etc/mi/cyclonedds.xml",
    "export DISPLAY=:0",
    "export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/usr/include/python3.6/",
    "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH",
    "export USER=mi"
  };                                              /*!< bash 配置 */
  std::string logger_ {""};                       /*!< 日志 */

protected:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};    /*!< 节点 */
  toml::value params_toml_;                       /*!< 配置文件数据 */
  StateEnum state_;                               /*!< 状态 */
  std::string describe_ {""};                     /*!< 描述 */
  std::string base_path_ {""};                    /*!< 任务路径 */
  std::vector<std::string> header_sh_;            /*!< shell 文件头 */
  std::string registry_fil_ {""};                 /*!< 注册表文件 */
  std::string registry_related_fil_ {""};         /*!< 相关注册表 */
  const std::string judge_python_ {"pyflakes "};  /*!< 判断 python */
  std::string type_ {""};                         /*!< 类型 */
  bool decorate_body_;                            /*!< 装饰身体 */

public:
  std::shared_ptr<PythonInterpreter>
  py_interpreter_ptr_ {nullptr};                  /*!< python 解释器 */

public:
  bool RespondToRequests(
    const OperateMsg &, GRPCMsg &);               /*!< 响应请求 */
  void getRobotendMsg(
    const OperateMsg &, GRPCMsg &);               /*!< 获取后端消息 */
  virtual bool ExecuteRequest(
    const OperateMsg &, GRPCMsg &);               /*!< 执行请求 */
  bool GetRegistryToml(
    toml::value &,
    bool _is_registry_file = true);               /*!< 获取注册表 */
  bool SetRegistryToml(
    const toml::value &,
    bool _is_registry_file = true);               /*!< 设置注册表 */
  bool GetState(
    const std::string,
    std::string &);                               /*!< 获取状态 */
  bool SetList(
    const OperateMsg &,
    const std::string &,
    const std::string &,
    const std::vector<std::string> &);            /*!< 设置列表 */
  bool GetList(const OperateMsg &, GRPCMsg &);    /*!< 获取列表 */
  bool GetMeta(
    const toml::value &,
    const std::string &, OperateMsg &);           /*!< 获取任务 */
  bool GetMeta(
    const std::string &, OperateMsg &);           /*!< 获取任务 */
  bool BuildFrontendOperate(
    const std::string,
    std::string &);                               /*!< 构建前端操作 */
  bool BuildBackendOperate(
    const OperateMsg &,
    std::string &);                               /*!< 构建后端消息(查询) */
};  // class Base
}  // namespace cyberdog_visual_programming_engine
#endif  // CYBERDOG_VP_ENGINE__BASE_HPP_
