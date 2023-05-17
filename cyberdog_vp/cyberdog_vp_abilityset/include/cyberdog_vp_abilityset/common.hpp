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
#ifndef CYBERDOG_VP_ABILITYSET__COMMON_HPP_
#define CYBERDOG_VP_ABILITYSET__COMMON_HPP_

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <cyberdog_common/cyberdog_json.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <protocol/msg/train_plan.hpp>
#include <protocol/msg/user_information.hpp>
#include <protocol/msg/face_recognition_result.hpp>
#include <protocol/msg/gesture_action_result.hpp>
#include <protocol/msg/motion_sequence_pace.hpp>
#include <protocol/msg/audio_play_extend.hpp>
#include <protocol/msg/audio_play.hpp>
#include <protocol/msg/connector_status.hpp>
#include <protocol/msg/motion_id.hpp>
#include <protocol/msg/bms_status.hpp>
#include <protocol/msg/touch_status.hpp>
#include <protocol/msg/head_tof_payload.hpp>
#include <protocol/msg/rear_tof_payload.hpp>
#include <protocol/msg/gps_payload.hpp>
#include <protocol/msg/single_tof_payload.hpp>
#include <protocol/msg/motion_servo_cmd.hpp>
#include <protocol/msg/motion_servo_response.hpp>
#include <protocol/msg/motion_status.hpp>
#include <protocol/msg/label.hpp>
#include <protocol/msg/sport_counts_result.hpp>
#include <protocol/msg/algo_task_status.hpp>
#include <protocol/msg/visual_programming_operate.hpp>

#include <protocol/srv/visual_programming_operate.hpp>
#include <protocol/srv/audio_text_play.hpp>
#include <protocol/srv/audio_volume_get.hpp>
#include <protocol/srv/audio_volume_set.hpp>
#include <protocol/srv/led_execute.hpp>
#include <protocol/srv/motion_result_cmd.hpp>
#include <protocol/srv/motion_sequence_show.hpp>
#include <protocol/srv/all_user_search.hpp>
#include <protocol/srv/face_rec.hpp>
#include <protocol/srv/gesture_action_control.hpp>
#include <protocol/srv/get_map_label.hpp>
#include <protocol/srv/stop_algo_task.hpp>
#include <protocol/srv/sport_manager.hpp>
#include <protocol/srv/train_plan_all.hpp>

#include <protocol/action/navigation.hpp>

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
#include <atomic>
#include <algorithm>
#include <cmath>

#include <condition_variable>
#include <thread>
#include <chrono>

namespace cyberdog_visual_programming_abilityset
{
/*! \file       common.hpp
    \brief      可视化编程参数模块。
    \details    定义参数约束及约束参数。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        依据参数列表均可直接调用。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/

/*             输入消息                   输出消息
 * +-->message>>(IMsg)>>[vp abilityset]>>(OMsg)>>message>--+
 * |_______________________________________________________|
 */
using CyberdogJson =
  cyberdog::common::CyberdogJson;                 /*!< JSON 解析及构建模块类型 */
using CyberdogToml =
  cyberdog::common::CyberdogToml;                 /*!< Toml 解析及构建模块类型 */

using MsgHeader = std_msgs::msg::Header;          /*!< 消息头 */
using MsgString = std_msgs::msg::String;          /*!< GRPC Voiceprint */
using MsgBool = std_msgs::msg::Bool;              /*!< Voiceprint */
using MsgLaserScan =
  sensor_msgs::msg::LaserScan;                    /*!< 激光数据 */
using MsgImu = sensor_msgs::msg::Imu;             /*!< IMU */
using MsgRange = sensor_msgs::msg::Range;         /*!< Range */
using MsgOdometry = nav_msgs::msg::Odometry;      /*!< 里程计 */
using MsgPose = geometry_msgs::msg::Pose;         /*!< 位姿 */
using MsgPoseStamped =
  geometry_msgs::msg::PoseStamped;                /*!< 位姿 */
using MsgPoint = geometry_msgs::msg::Point;       /*!< 位置 */
using MsgMotionID = protocol::msg::MotionID;      /*!< 运动 id */
using MsgMotionServoCmd =
  protocol::msg::MotionServoCmd;                  /*!< 伺服指令 */
using MsgMotionStatus =
  protocol::msg::MotionStatus;                    /*!< 运动状态 */
using MsgSingleTofPayload =
  protocol::msg::SingleTofPayload;                /*!< 单个tof数据 */
using MsgHeadTofPayload =
  protocol::msg::HeadTofPayload;                  /*!< 头部tof数据 */
using MsgRearTofPayload =
  protocol::msg::RearTofPayload;                  /*!< 尾部tof数据 */
using MsgGpsPayload = protocol::msg::GpsPayload;  /*!< GPS数据 */
using MsgAudioPlayExtend =
  protocol::msg::AudioPlayExtend;                 /*!< 语音消息:在线 */
using MsgAudioPlay = protocol::msg::AudioPlay;    /*!< 语音消息:离线 */
using MsgConnectorStatus =
  protocol::msg::ConnectorStatus;                 /*!< 连接状态 */
using MsgMotionServoResponse =
  protocol::msg::MotionServoResponse;             /*!< 伺服指令 */
using MsgMotionSequenceGait =
  protocol::msg::MotionSequenceGait;              /*!< 运动序列步态 */
using MsgMotionSequencePace =
  protocol::msg::MotionSequencePace;              /*!< 运动序列步伐 */
using MsgTouchStatus =
  protocol::msg::TouchStatus;                     /*!< 触摸板 */
using MsgBmsStatus = protocol::msg::BmsStatus;    /*!< 电池系统 */
using MsgPersonnel =
  protocol::msg::UserInformation;                 /*!< 人员信息 */
using MsgFaceRes =
  protocol::msg::FaceRecognitionResult;           /*!< 人脸识别反馈 */
using MsgGesture =
  protocol::msg::GestureActionResult;             /*!< 手势信息 */
using MsgSport =
  protocol::msg::SportCountsResult;               /*!< 骨骼点 消息 */
using MsgPreset = protocol::msg::Label;           /*!< 预置点 消息 */
using MsgTrainingWords =
  protocol::msg::TrainPlan;                       /*!< 训练词 */
using MsgAlgoStatus =
  protocol::msg::AlgoTaskStatus;                  /*!< 算法状态 */
using MsgVisualProgrammingOperate =
  protocol::msg::VisualProgrammingOperate;        /*!< 任务操作消息 */

using SrvVisualProgrammingOperate =
  protocol::srv::VisualProgrammingOperate;        /*!< 任务操作服务 */
using SrvTrainingWords =
  protocol::srv::TrainPlanAll;                    /*!< 训练词服务 */
using SrvPersonnel =
  protocol::srv::AllUserSearch;                   /*!< 人员服务 */
using SrvFaceRec = protocol::srv::FaceRec;        /*!< 人脸识别服务 */
using SrvGesture =
  protocol::srv::GestureActionControl;            /*!< 手势服务 */
using SrvSport = protocol::srv::SportManager;     /*!< 骨骼点服务 */
using SrvMotionSequenceShow =
  protocol::srv::MotionSequenceShow;              /*!< 序列动作 */
using SrvMotionResultCmd =
  protocol::srv::MotionResultCmd;                 /*!< 结果指令 */
using SrvLedExecute = protocol::srv::LedExecute;  /*!< led */
using SrvAudioTextPlay =
  protocol::srv::AudioTextPlay;                   /*!< audio 播放 */
using SrvAudioGetVolume =
  protocol::srv::AudioVolumeGet;                  /*!< audio 音量获取 */
using SrvAudioSetVolume =
  protocol::srv::AudioVolumeSet;                  /*!< audio 音量设置 */
using SrvGetPreset = protocol::srv::GetMapLabel;  /*!< audio 获取预置点 */
using SrvCancelNavigation =
  protocol::srv::StopAlgoTask;                    /*!< 取消导航 */
using SrvSetBool = std_srvs::srv::SetBool;        /*!< bool类型 */

using ActNavigation =
  protocol::action::Navigation;                   /*!< 导航动作 */

static const rclcpp::QoS SubscriptionSensorQos =
  rclcpp::SensorDataQoS();                        /*!< [质量服务]监听器:传感器 */
static const rclcpp::QoS SubscriptionQos =
  rclcpp::ParametersQoS();                        /*!< [质量服务]监听器 */
static const rclcpp::QoS PublisherQos =
  rclcpp::ParametersQoS();                        /*!< [质量服务]发布器 */
static const rclcpp::QoS ClientQos =
  rclcpp::SystemDefaultsQoS();                    /*!< [质量服务]客户端 */
static const rclcpp::QoS ServicesQos =
  rclcpp::ServicesQoS();                          /*!< [质量服务]服务端 */

#define STREAM(msg)(__extension__( \
    {std::ostringstream nowmsg;nowmsg << msg; \
      const char * result = nowmsg.str().c_str();(result);}));

#define FORMAT(format, ...)(__extension__( \
    {int size = snprintf(NULL, 0, format, ## __VA_ARGS__);size++; \
      char * result = new char[size]; \
      snprintf(result, size, format, ## __VA_ARGS__);(result);}))

#define ONE_FORMAT(format)(__extension__( \
    {int size = snprintf(NULL, 0, format);size++; \
      char * result = new char[size]; \
      snprintf(result, size, format);(result);}))

template<typename ... Args>
std::string Format(const char * format, Args... args)
{
  std::string str = "";
  int size = std::snprintf(nullptr, 0, format, args ...);
  if (size > 0) {
    size++;
    char * message = nullptr;
    message = new(std::nothrow) char[size];
    if (message != nullptr) {
      std::snprintf(message, size, format, args ...);
      str = std::string(message);
      delete[] message;
      message = nullptr;
    }
  }
  return str;
}

static const double pi = 3.14159265;              /*!< 弧度转角度 */
static const double rad2ang = 180 / pi;           /*!< 弧度转角度 */

/*! 参数约束:默认值和最大值 */
struct DefaultAndMaximum
{
  double minimum_value;                           /*!< 最小值 */
  double default_value;                           /*!< 默认值 */
  double maximum_value;                           /*!< 最大值 */
  std::string unit;                               /*!< 单位 */
};

/*! 参数约束:运动 */
static struct MotionParams
{
  DefaultAndMaximum centroid_x =
  {-0.4, 0, 0.4, "m"};                            /*!< [类型]质心X轴约束 */
  DefaultAndMaximum centroid_y =
  {-0.3, 0, 0.3, "m"};                            /*!< [类型]质心Y轴约束 */
  DefaultAndMaximum centroid_z =
  {-0.3, 0.2, 0.3, "m"};                          /*!< [类型]质心Z轴约束 */
  DefaultAndMaximum fulcrum_x =
  {-0.5, 0, 0.5, "m"};                            /*!< [类型]支点X轴约束 */
  DefaultAndMaximum fulcrum_y =
  {-0.5, 0, 0.5, "m"};                            /*!< [类型]支点Y轴约束 */
  DefaultAndMaximum fulcrum_z =
  {-0.5, 0, 0.5, "m"};                            /*!< [类型]支点Z轴约束 */
  DefaultAndMaximum roll =
  {-0.45 * rad2ang, 0, 0.45 * rad2ang, "deg"};    /*!< [类型]机身翻滚 */
  DefaultAndMaximum pitch =
  {-0.45 * rad2ang, 0, 0.45 * rad2ang, "deg"};    /*!< [类型]机身俯仰 */
  DefaultAndMaximum yaw =
  {-0.45 * rad2ang, 0, 0.45 * rad2ang, "deg"};    /*!< [类型]机身偏航 */
  DefaultAndMaximum x_velocity =
  {-1.6, 0.3, 1.6, "m/s"};                        /*!< [类型]x速度 */
  DefaultAndMaximum y_velocity =
  {-1.2, 0.3, 1.2, "m/s"};                        /*!< [类型]y速度 */
  DefaultAndMaximum z_velocity =
  {-2.5, 1.0 * rad2ang, 2.5 * rad2ang, "deg/s"};  /*!< [类型]z角速度 */
  DefaultAndMaximum front_leg_lift =
  {0.01, 0.03, 0.1, "m"};                         /*!< [类型]前腿抬腿高度 */
  DefaultAndMaximum back_leg_lift =
  {0.01, 0.03, 0.1, "m"};                         /*!< [类型]后腿抬腿高度 */
  DefaultAndMaximum duration =
  {0.0, 0.0, 3600, "s"};                          /*!< [类型]期望时间 */
  DefaultAndMaximum distance =
  {0.0, 0.0, 1000, "m"};                          /*!< [类型]期望距离 */
  DefaultAndMaximum compensation_frame =
  {0.0, 0.0, 10, "n"};                            /*!< [类型]补偿帧 */
  DefaultAndMaximum reserved =
  {0.0, 0.0, 1000, "x"};                          /*!< [类型]预留 */
  DefaultAndMaximum delta =
  {0.1, 10, 100, "per"};                          /*!< [类型]变化量 */
} motion_params_;

/*! 处理器类型 */
enum Processor
{
  task                          = 0,              /*!< 任务 */
};

/*! 运动ID
  0.急停
  1.趴下
  2.位控姿态
  3.力控姿态
  4.运动中 */
enum MotionId
{
  emergency_stop                          = 0,    /*!< 急停 */

  get_down                                = 101,  /*!< [1]趴下 */
  resume_standing                         = 111,  /*!< [2]恢复站立 */
  servo_standing                          = 112,  /*!< [3]伺服站立(伺服指令END帧后状态) */
  back_flip                               = 121,  /*!< [2]后空翻 */
  front_flip                              = 122,  /*!< [2]前空翻 */
  bow                                     = 123,  /*!< [2]作揖 */
  roll_left                               = 124,  /*!< [2]向左侧躺后恢复 */
  walk_the_dog                            = 125,  /*!< [4]遛狗 */
  jump_stair                              = 126,  /*!< [2]跳上台阶 */
  right_somersault                        = 127,  /*!< [2]右侧空翻 */
  left_somersault                         = 128,  /*!< [2]左侧空翻 */
  run_and_jump_front_flip                 = 129,  /*!< [2]跑跳前空翻 */
  jump3d_left90deg                        = 130,  /*!< [2]3D跳:左转90度 */
  jump3d_right90deg                       = 131,  /*!< [2]3D跳:右转90度 */
  jump3d_forward60cm                      = 132,  /*!< [2]3D跳:前跳60cm */
  jump3d_forward30cm                      = 133,  /*!< [2]3D跳:前跳30cm */
  jump3d_left20cm                         = 134,  /*!< [2]3D跳:左跳20cm */
  jump3d_right20cm                        = 135,  /*!< [2]3D跳:右跳20cm */
  jump3d_up30cm                           = 136,  /*!< [2]3D跳:向上30cm */
  jump3d_down_stair                       = 137,  /*!< [2]3D跳:跳下台阶 */
  roll_right                              = 138,  /*!< [2]向右侧躺后恢复 */
  dance_collection                        = 140,  /*!< [2]舞蹈集合 */
  hold_left_hand                          = 141,  /*!< [2]握左手 */
  hold_right_hand                         = 142,  /*!< [2]握右手 */
  sit_down                                = 143,  /*!< [2]坐下 */
  butt_circle                             = 144,  /*!< [2]屁股画圆 */
  head_circle                             = 145,  /*!< [2]头画圆 */
  stretch_the_body                        = 146,  /*!< [2]伸展身体 */
  shake_ass_left                          = 148,  /*!< [2]向左摇晃屁股 */
  shake_ass_right                         = 149,  /*!< [2]向右摇晃屁股 */
  shake_ass_from_side_to_side             = 150,  /*!< [2]左右摇晃屁股 */
  ballet                                  = 151,  /*!< [3]芭蕾舞 */
  space_walk                              = 152,  /*!< [3]太空步 */
  front_leg_jumping                       = 153,  /*!< [3]前腿开合跳 */
  hind_leg_jumping                        = 154,  /*!< [3]后腿开合跳 */
  lift_the_left_leg_and_nod               = 155,  /*!< [3]左腿抬起并点头 */
  lift_the_right_leg_and_nod              = 156,  /*!< [3]右腿抬起并点头 */
  left_front_right_back_legs_apart        = 157,  /*!< [3]左前右后岔开腿 */
  right_front_left_back_legs_apart        = 158,  /*!< [3]右前左后岔开腿 */
  walk_nodding                            = 159,  /*!< [3]走路点头 */
  walking_with_divergence_and_adduction_alternately =
    160,                                          /*!< [3]岔开内收交替走路 */
  nodding_in_place                        = 161,  /*!< [3]原地踏步点头 */
  front_legs_jump_back_and_forth          = 162,  /*!< [3]前腿前后跳 */
  hind_legs_jump_back_and_forth           = 163,  /*!< [3]后腿前后跳 */
  alternately_front_leg_lift              = 164,  /*!< [3]前腿交替抬起 */
  alternately_hind_leg_lift               = 165,  /*!< [3]后腿交替抬起 */
  jump_collection                         = 166,  /*!< [3]跳跃合集 */
  stretching_left_and_right               = 167,  /*!< [3]左右伸腿踏步 */
  jump_forward_and_backward               = 168,  /*!< [3]前后摆腿跳跃 */
  step_left_and_right                     = 169,  /*!< [3]左右摆腿踏步 */
  right_leg_back_and_forth_stepping       = 170,  /*!< [3]右腿前后踏步 */
  left_leg_back_and_forth_stepping        = 171,  /*!< [3]左腿前后踏步 */
  squat_down_on_all_fours                 = 173,  /*!< [2]四足蹲起 */
  push_ups                                = 174,  /*!< [2]俯卧撑 */
  bow_to_each_other                       = 175,  /*!< [2]作揖比心 */

  absolute_force_control_attitude         = 201,  /*!< [3]绝对力控姿态（绝对姿态） */
  relatively_force_control_attitude       = 202,  /*!< [3]相对力控姿态 */
  absolute_position_control_attitude      = 211,  /*!< [2]绝对位控姿态（过渡站立） */
  relatively_position_control_attitude    = 212,  /*!< [2]相对位控姿态 */
  relatively_position_control_attitude_insert_frame_1 =
    213,                                          /*!< [2]相对位姿插帧1(握手) */
  relatively_position_control_attitude_insert_frame_2 =
    214,                                          /*!< [2]相对位姿插帧2(任务空时) */

  jump_back_and_forth                     = 301,  /*!< [3]前后跳 */
  small_jump_walking                      = 302,  /*!< [3]小跳行走 */
  trot_walking                            = 303,  /*!< [3]慢速（小跑）行走 */
  automatic_frequency_conversion_walking  = 304,  /*!< [3]自动变频行走 */
  run_fast_walking                        = 305,  /*!< [3]快跑行走 */
  sequence_custom                         = 400,  /*!< [3]序列动作 */

  illegal_motion_id                       = 999,  /*!< 非法运动ID */
};

/*! 通用状态约束:{内部错误码 = 错误码 - 基础码}
  错误基础码: 5800
  内部错误码:
    01~20:全局;
    21~30:模块;
    31~40:ROS;
    41~50:Topic;
    51~60:Service;
    61~70:Action;
*/
enum StateCode
{
  invalid                               = -1,     /*!< 无效 */
  success                               = 0,      /*!< 成功 */
  error_base                            = 5800,   /*!< 错误基础码 */
  fail                                  = 5801,   /*!< [ 全局码 ]失败 */
  uninitialized                         = 5802,   /*!< [ 全局码 ]未初始化 */
  fsm_does_not_allow                    = 5803,   /*!< [ 全局码 ]状态机不允许 */
  module_status_error                   = 5804,   /*!< [ 全局码 ]模块状态错误 */
  network_error                         = 5805,   /*!< [ 全局码 ]网络错误 */
  no_operation_authority                = 5806,   /*!< [ 全局码 ]无操作权限 */
  timeout                               = 5807,   /*!< [ 全局码 ]超时 */
  command_does_not_support              = 5808,   /*!< [ 全局码 ]指令不支持 */
  self_test_failed                      = 5809,   /*!< [ 全局码 ]自检失败 */
  parameter_is_invalid                  = 5810,   /*!< [ 全局码 ]参数不合法 */
  status_is_busy                        = 5811,   /*!< [ 全局码 ]状态忙碌 */
  hardware_error                        = 5812,   /*!< [ 全局码 ]硬件错误 */
  command_waiting_execute               = 5821,   /*!< [ 模块码 ]命令等待执行 */
  spin_future_interrupted               = 5831,   /*!< [  ROS  ]请求服务中断 */
  spin_future_timeout                   = 5832,   /*!< [  ROS  ]请求服务超时/延迟 */
  no_data_update                        = 5841,   /*!< [ Topic ]无数据更新 */
  service_client_interrupted            = 5851,   /*!< [Service]客户端在请求服务出现时被打断 */
  service_appear_timeout                = 5852,   /*!< [Service]等待服务出现（启动）超时 */
  service_request_interrupted           = 5853,   /*!< [Service]请求服务中断 */
  service_request_rejected              = 5854,   /*!< [Service]请求服务被拒绝 */
  service_request_timeout               = 5855,   /*!< [Service]请求服务超时/延迟 */
  action_request_timeout                = 5861,   /*!< [Action ]请求动作超时/延迟 */
  action_request_rejected               = 5862,   /*!< [Action ]请求动作被拒绝 */
  action_result_timeout                 = 5863,   /*!< [Action ]等待动作结果超时/延迟 */
};

static std::unordered_map<StateCode, std::string> StateDescribe_ = {
  {StateCode::invalid, "invalid"},
  {StateCode::success, "success"},
  {StateCode::error_base, "error_base"},
  {StateCode::fail, "fail"},
  {StateCode::uninitialized, "uninitialized"},
  {StateCode::fsm_does_not_allow, "fsm does not allow"},
  {StateCode::module_status_error, "module status error"},
  {StateCode::network_error, "network error"},
  {StateCode::no_operation_authority, "no operation authority"},
  {StateCode::timeout, "timeout"},
  {StateCode::command_does_not_support, "command does not support"},
  {StateCode::self_test_failed, "self test failed"},
  {StateCode::parameter_is_invalid, "parameter is invalid"},
  {StateCode::status_is_busy, "status is busy"},
  {StateCode::hardware_error, "hardware error"},
  {StateCode::command_waiting_execute, "command waiting execute"},
  {StateCode::spin_future_interrupted, "spin future interrupted"},
  {StateCode::spin_future_timeout, "spin future timeout"},
  {StateCode::no_data_update, "no data update"},
  {StateCode::service_client_interrupted, "service client interrupted"},
  {StateCode::service_appear_timeout, "service appear timeout"},
  {StateCode::service_request_interrupted, "service request interrupted"},
  {StateCode::service_request_rejected, "service request rejected"},
  {StateCode::service_request_timeout, "service request timeout"},
  {StateCode::action_request_timeout, "action request timeout"},
  {StateCode::action_request_rejected, "action request rejected"},
  {StateCode::action_result_timeout, "action result timeout"},
};                                                /*! 通用状态描述 */

/*! 欧拉角约束调用 角度类型 合法值 */
enum RPYType
{
  ROLL = 0,                                       /*!< 横滚 */
  PITCH,                                          /*!< 俯仰 */
  YAW                                             /*!< 偏航 */
};

/*! 欧拉角约束调用 */
struct RPY
{
  double roll = 0.0;                              /*!< 横滚 */
  double pitch = 0.0;                             /*!< 俯仰 */
  double yaw = 0.0;                               /*!< 偏航 */
};

/*! 障碍物 */
struct ObstacleMeta
{
  bool detected = false;                          /*!< 检测到 */
  std::chrono::time_point<std::chrono::system_clock>
  validity_period_time;                           /*!< 有效期时间 */
};

/*! 障碍物 */
struct TofObstacle
{
  ObstacleMeta head_left;                         /*!< 头部左侧 */
  ObstacleMeta head_right;                        /*!< 头部右侧 */
  ObstacleMeta rear_left;                         /*!< 尾部左侧 */
  ObstacleMeta rear_right;                        /*!< 尾部右侧 */
};

/*! 参数约束:tof数据 */
class TofPayload
{
public:
  TofPayload() {}
  ~TofPayload() {}
  MsgHeadTofPayload head;                         /*!< 头部数据 */
  MsgRearTofPayload rear;                         /*!< 尾部数据 */
};

/*! 参数约束:默认值和最大值 */
class State
{
public:
  State()
  {
    code = StateCode::invalid;
    describe = StateDescribe_[StateCode::invalid];
  }
  ~State() {}
  StateCode code;                                 /*!< 状态 */
  std::string describe;                           /*!< 描述 */
};

/*! 运动结果指令服务反馈 */
class MotionResultServiceResponse
{
public:
  MotionResultServiceResponse() {}
  ~MotionResultServiceResponse() {}
  State state;                                    /*!< 状态 */
  SrvMotionResultCmd::Response response;          /*!< 反馈 */
};

/*! 运动序列动作服务反馈 */
class MotionSequenceServiceResponse
{
public:
  MotionSequenceServiceResponse() {}
  ~MotionSequenceServiceResponse() {}
  State state;                                    /*!< 状态 */
  SrvMotionSequenceShow::Response response;       /*!< 反馈 */
};

/*! 运动伺服反馈 */
class MotionServoCmdResponse
{
public:
  MotionServoCmdResponse() {}
  ~MotionServoCmdResponse() {}
  State state;                                    /*!< 状态 */
  MsgMotionServoResponse response;                /*!< 反馈 */
};

/*! audio 播放服务反馈 */
class AudioPlaySeviceResponse
{
public:
  AudioPlaySeviceResponse() {response.status = 100;}
  ~AudioPlaySeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvAudioTextPlay::Response response;            /*!< 反馈 */
};

/*! audio 获取音量服务反馈 */
class AudioGetVolumeSeviceResponse
{
public:
  AudioGetVolumeSeviceResponse() {response.volume = 0;}
  ~AudioGetVolumeSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvAudioGetVolume::Response response;           /*!< 反馈 */
};

/*! audio 获取音量服务反馈 */
class AudioSetVolumeSeviceResponse
{
public:
  AudioSetVolumeSeviceResponse() {response.success = false;}
  ~AudioSetVolumeSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvAudioSetVolume::Response response;           /*!< 反馈 */
};

/*! audio 获取用户对话 */
class DialogueResponse
{
public:
  DialogueResponse() {}
  ~DialogueResponse() {}
  uint64_t time_ns;                               /*!< 时间 */
  std::string data;                               /*!< 数据 */
};

/*! audio 获取用户对话 */
class AudioGetUserDialogueResponse
{
public:
  AudioGetUserDialogueResponse() {}
  ~AudioGetUserDialogueResponse() {}
  State state;                                    /*!< 状态 */
  std::vector<DialogueResponse> response;         /*!< 反馈 */
};


/*! LED约束 */
enum LedConstraint
{
  target_head =
    SrvLedExecute::Request::HEAD_LED,             /*!< 头灯 */
  target_tail =
    SrvLedExecute::Request::TAIL_LED,             /*!< 尾灯 */
  target_mini =
    SrvLedExecute::Request::MINI_LED,             /*!< 眼灯 */

  effect_line_on =
    SrvLedExecute::Request::RGB_ON,               /*!< [灯带]常亮 */
  effect_line_blink =
    SrvLedExecute::Request::BLINK,                /*!< [灯带]闪烁 */
  effect_line_blink_fast =
    SrvLedExecute::Request::BLINK_FAST,           /*!< [灯带]快速闪烁 */
  effect_line_breath =
    SrvLedExecute::Request::BREATH,               /*!< [灯带]呼吸 */
  effect_line_breath_fast =
    SrvLedExecute::Request::BREATH_FAST,          /*!< [灯带]快速呼吸 */
  effect_line_one_by_one =
    SrvLedExecute::Request::ONE_BY_ONE,           /*!< [灯带]逐个点亮 */
  effect_line_one_by_one_fast =
    SrvLedExecute::Request::ONE_BY_ONE_FAST,      /*!< [灯带]快速逐个点亮 */
  effect_line_back_and_forth =
    SrvLedExecute::Request::BACK_AND_FORTH,       /*!< [灯带]往返逐个点亮 */
  effect_line_trailing_race =
    SrvLedExecute::Request::TRAILING_RACE,        /*!< [灯带]拖尾流跑马 */

  system_effect_line_off =
    SrvLedExecute::Request::RGB_OFF,              /*!< [灯带]常灭 */

  system_effect_line_red_on =
    SrvLedExecute::Request::RED_ON,               /*!< [灯带]红灯常亮 */
  system_effect_line_red_blink =
    SrvLedExecute::Request::RED_BLINK,            /*!< [灯带]红灯闪烁 */
  system_effect_line_red_blink_fast =
    SrvLedExecute::Request::RED_BLINK_FAST,       /*!< [灯带]红灯快速闪烁 */
  system_effect_line_red_breath =
    SrvLedExecute::Request::RED_BREATH,           /*!< [灯带]红灯呼吸 */
  system_effect_line_red_breath_fast =
    SrvLedExecute::Request::RED_BREATH_FAST,      /*!< [灯带]红灯快速呼吸 */
  system_effect_line_red_one_by_one =
    SrvLedExecute::Request::RED_ONE_BY_ONE,       /*!< [灯带]红灯逐个点亮 */
  system_effect_line_red_one_by_one_fast =
    SrvLedExecute::Request::RED_ONE_BY_ONE_FAST,  /*!< [灯带]红灯快速逐个点亮 */

  system_effect_line_blue_on =
    SrvLedExecute::Request::BLUE_ON,              /*!< [灯带]红灯常亮 */
  system_effect_line_blue_blink =
    SrvLedExecute::Request::BLUE_BLINK,           /*!< [灯带]红灯闪烁 */
  system_effect_line_blue_blink_fast =
    SrvLedExecute::Request::BLUE_BLINK_FAST,      /*!< [灯带]红灯快速闪烁 */
  system_effect_line_blue_breath =
    SrvLedExecute::Request::BLUE_BREATH,          /*!< [灯带]红灯呼吸 */
  system_effect_line_blue_breath_fast =
    SrvLedExecute::Request::BLUE_BREATH_FAST,     /*!< [灯带]红灯快速呼吸 */
  system_effect_line_blue_one_by_one =
    SrvLedExecute::Request::BLUE_ONE_BY_ONE,      /*!< [灯带]红灯逐个点亮 */
  system_effect_line_blue_one_by_one_fast =
    SrvLedExecute::Request::BLUE_ONE_BY_ONE_FAST, /*!< [灯带]红灯快速逐个点亮 */

  system_effect_line_yellow_on =
    SrvLedExecute::Request::YELLOW_ON,            /*!< [灯带]红灯常亮 */
  system_effect_line_yellow_blink =
    SrvLedExecute::Request::YELLOW_BLINK,         /*!< [灯带]红灯闪烁 */
  system_effect_line_yellow_blink_fast =
    SrvLedExecute::Request::YELLOW_BLINK_FAST,    /*!< [灯带]红灯快速闪烁 */
  system_effect_line_yellow_breath =
    SrvLedExecute::Request::YELLOW_BREATH,        /*!< [灯带]红灯呼吸 */
  system_effect_line_yellow_breath_fast =
    SrvLedExecute::Request::YELLOW_BREATH_FAST,   /*!< [灯带]红灯快速呼吸 */
  system_effect_line_yellow_one_by_one =
    SrvLedExecute::Request::YELLOW_ONE_BY_ONE,    /*!< [灯带]红灯逐个点亮 */
  system_effect_line_yellow_one_by_one_fast =     /*!< [灯带]红灯快速逐个点亮 */
    SrvLedExecute::Request::YELLOW_ONE_BY_ONE_FAST,

  effect_mini_circular_breath =
    SrvLedExecute::Request::CIRCULAR_BREATH,      /*!< [眼灯]圆形缩放 */
  effect_mini_circular_ring =
    SrvLedExecute::Request::CIRCULAR_RING,        /*!< [眼灯]画圆环 */

  system_effect_mini_off =
    SrvLedExecute::Request::MINI_OFF,             /*!< [眼灯]常灭 */

  system_effect_mini_rectangle_color =
    SrvLedExecute::Request::RECTANGLE_COLOR,      /*!< [眼灯]方块变色 */
  system_effect_mini_centre_color =
    SrvLedExecute::Request::CENTRE_COLOR,         /*!< [眼灯]中间彩带 */
  system_effect_mini_three_circular =
    SrvLedExecute::Request::THREE_CIRCULAR,       /*!< [眼灯]三圆呼吸 */
  system_effect_mini_one_by_one =
    SrvLedExecute::Request::COLOR_ONE_BY_ONE,     /*!< [眼灯]彩带逐个点亮 */
};

/*! led服务反馈 */
class LedSeviceResponse
{
public:
  LedSeviceResponse() {}
  ~LedSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvLedExecute::Response response;               /*!< 反馈 */
};

/*! 人脸识别服务反馈 */
class FaceSeviceResponse
{
public:
  FaceSeviceResponse() {}
  ~FaceSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvFaceRec::Response response;                  /*!< 反馈 */
};

/*! 人脸识别服务反馈 */
class FaceRecognizedSeviceResponse
{
public:
  FaceRecognizedSeviceResponse() {}
  ~FaceRecognizedSeviceResponse() {}
  State state;                                    /*!< 状态 */
  std::vector<MsgFaceRes> list;                   /*!< 列表 */
  std::map<std::string, MsgFaceRes> dictionary;   /*!< 字典 */
};

/*! 声纹识别返回信息 */
class VoiceprintRecognizedResponse
{
public:
  VoiceprintRecognizedResponse() {}
  ~VoiceprintRecognizedResponse() {}
  State state;                                    /*!< 状态 */
  std::vector<std::string> list;                  /*!< 反馈 */
  bool data;                                      /*!< 数据 */
};

/*! 手势种类 */
enum GestureType
{
  no_gesture                              = 0,    /*!< 无手势 */
  pulling_hand_or_two_fingers_in          = 1,    /*!< 手掌拉近 */
  pushing_hand_or_two_fingers_away        = 2,    /*!< 手掌推开 */
  sliding_hand_or_two_fingers_up          = 3,    /*!< 手向上抬 */
  sliding_hand_or_two_fingers_down        = 4,    /*!< 手向下压 */
  sliding_hand_or_two_fingers_left        = 5,    /*!< 手向左推 */
  sliding_hand_or_two_fingers_right       = 6,    /*!< 手向右推 */
  stop_sign                               = 7,    /*!< 停止手势 */
  thumb_up                                = 8,    /*!< 大拇指朝上 */
  zooming_in_with_hand_or_two_fingers     = 9,    /*!< 张开手掌或手指 */
  zooming_out_with_hand_or_two_fingers    = 10,   /*!< 闭合手掌或手指 */
  thumb_down                              = 11,   /*!< 大拇指朝下 */
  id_upper_limit                          = 12,   /*!< 手势id上限 */
};

/*! 手势数据 */
struct GestureData
{
  bool pulling_hand_or_two_fingers_in = false;    /*!< 手掌拉近 */
  bool pushing_hand_or_two_fingers_away = false;  /*!< 手掌推开 */
  bool sliding_hand_or_two_fingers_up = false;    /*!< 手向上抬 */
  bool sliding_hand_or_two_fingers_down = false;  /*!< 手向下压 */
  bool sliding_hand_or_two_fingers_left = false;  /*!< 手向左推 */
  bool sliding_hand_or_two_fingers_right = false; /*!< 手向右推 */
  bool stop_sign = false;                         /*!< 停止手势 */
  bool thumb_down = false;                        /*!< 大拇指朝下 */
  bool thumb_up = false;                          /*!< 大拇指朝上 */
  bool zooming_in_with_hand_or_two_fingers =
    false;                                        /*!< 张开手掌或手指 */
  bool zooming_out_with_hand_or_two_fingers =
    false;                                        /*!< 闭合手掌或手指 */
};

/*! 手势识别服务反馈 */
class GestureRecognizedSeviceResponse
{
public:
  GestureRecognizedSeviceResponse() {}
  ~GestureRecognizedSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvGesture::Response response;                  /*!< 反馈 */
};

/*! 手势识别返回信息 */
class GestureRecognizedMessageResponse
{
public:
  GestureRecognizedMessageResponse() {}
  ~GestureRecognizedMessageResponse() {}
  State state;                                    /*!< 状态 */
  GestureData data;                               /*!< 数据 */
};

/*! 骨骼点识别类型 合法值 */
enum SkeletonType
{
  SPORT_SQUAT = 1,                                /*!< 深蹲 */
  SPORT_HIGHKNEES,                                /*!< 高抬腿 */
  SPORT_SITUP,                                    /*!< 仰卧起坐 */
  SPORT_PRESSUP,                                  /*!< 俯卧撑 */
  SPORT_PLANK,                                    /*!< 平板支撑 */
  SPORT_JUMPJACK                                  /*!< 开合跳 */
};

/*! 骨骼点识别服务反馈 */
class SkeletonRecognizedSeviceResponse
{
public:
  SkeletonRecognizedSeviceResponse() {}
  ~SkeletonRecognizedSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvSport::Response response;                    /*!< 反馈 */
};

/*! 骨骼点识别消息反馈 */
class SkeletonRecognizedMessageResponse
{
public:
  SkeletonRecognizedMessageResponse() {}
  ~SkeletonRecognizedMessageResponse() {}
  State state;                                    /*!< 状态 */
  MsgSport response;                              /*!< 反馈 */
};

/*! 训练词识别服务反馈 */
class TrainingWordsRecognizedSeviceResponse
{
public:
  TrainingWordsRecognizedSeviceResponse() {}
  ~TrainingWordsRecognizedSeviceResponse() {}
  State state;                                    /*!< 状态 */
  SrvTrainingWords::Response response;            /*!< 反馈 */
  std::map<std::string, MsgTrainingWords>
  dictionary;                                     /*!< 字典 */
};

/*! 训练词识别消息反馈 */
class TrainingWordsRecognizedMessageResponse
{
public:
  TrainingWordsRecognizedMessageResponse() {}
  ~TrainingWordsRecognizedMessageResponse() {}
  State state;                                    /*!< 状态 */
  MsgTrainingWords response;                      /*!< 反馈 */
};

/*! 预置点信息 */
class MapPresetSeviceResponse
{
public:
  MapPresetSeviceResponse() {}
  ~MapPresetSeviceResponse() {}
  State state;                                    /*!< 状态 */
  std::string map_name;                           /*!< 地图名 */
  bool is_outdoor;                                /*!< 是否为室外地图 */
  std::vector<MsgPreset> list;                    /*!< 列表 */
  std::map<std::string, MsgPreset> dictionary;    /*!< 字典 */
};

/*! 导航反馈信息 */
class NavigationActionResponse
{
public:
  NavigationActionResponse() {}
  ~NavigationActionResponse() {}
  State state;                                    /*!< 状态 */
  ActNavigation::Result response;                 /*!< 反馈 */
};

/*! 运动序列 */
class MotionSequence
{
public:
  MotionSequence() {}
  ~MotionSequence() {}
  std::string name;                               /*!< 名称 */
  std::string describe;                           /*!< 描述 */
  std::vector<MsgMotionSequenceGait> gait_list;   /*!< 步态列表 */
  std::vector<MsgMotionSequencePace> pace_list;   /*!< 步伐列表 */
};

/*! 约束获取时间接口入参的合法值 */
enum TimeMode
{
  Ms1970 = 0,                                     /*!< 1970年1月1日到现在的时间(毫秒) */
  _Y_M_D_H_M_S,                                   /*!< "*Y*M*D*H*M*S" */
  STANDARD,                                       /*!< "*Y.*M.*D-*H:*M:*S" */
  DETAILED,                                       /*!< "*Y.*M.*D-*H:*M:*S-US" */
};

geometry_msgs::msg::Quaternion RPY2Qrientation(
  const double, const double, const double);      /*!< 欧拉角转四元数 */
geometry_msgs::msg::Vector3 Qrientation2RPY(
  const double, const double, const double,
  const double);                                  /*!< 四元数转欧拉角 */
std::string GetTime(
  int nowModo = 0);                               /*!< 获取时间戳 */
double Angle2Radian(const double _degree);        /*!< 角度转换为弧度 */
double Radian2Angle(const double _rad);           /*!< 弧度转换为角度 */
uint64_t GetTimeNs();                             /*!< 获取时间 */
bool JudgeToml(const std::string &);              /*!< 判断 toml */
bool GetWorkspace(std::string &);                 /*!< 获取工作空间 */
bool Timeout(
  const uint64_t & _old_ns,
  uint64_t _timeout_ms = 3000);                   /*!< 判断超时 */
std::string int2binary(const int);                /*!< int 转 2进制 */
bool endsWith(
  const std::string &,
  const std::string &);                            /*!< 判断结束字符 */
std::string covariance36(
  const std::array<double, 36> &,
  const std::string,
  const int line_size = 6);                       /*!< 数组 转 字符串 */
std::string covariance9(
  const std::array<double, 9> &,
  const std::string,
  const int line_size = 3);                       /*!< 数组 转 字符串 */
std::string sequenceGaitVector(
  const std::vector<MsgMotionSequenceGait> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string sequencePaceVector(
  const std::vector<MsgMotionSequencePace> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgFaceResVector(
  const std::vector<MsgFaceRes> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgPersonnelVector(
  const std::vector<MsgPersonnel> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgFaceResMap(
  const std::map<std::string, MsgFaceRes> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgPresetVector(
  const std::vector<MsgPreset> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgPresetMap(
  const std::map<std::string, MsgPreset> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgTrainingWordsVector(
  const std::vector<MsgTrainingWords> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgTrainingWordsMap(
  const std::map<std::string, MsgTrainingWords> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string msgDialogueResponseVector(
  const std::vector<DialogueResponse> &,
  const std::string);                             /*!< 数组 转 字符串 */
std::string stringVector(
  const std::vector<std::string> &);              /*!< 数组 转 字符串 */
std::string intVectorToString(
  const std::vector<int> &);                      /*!< 数组 转 字符串 */
}  // namespace cyberdog_visual_programming_abilityset
#endif  // CYBERDOG_VP_ABILITYSET__COMMON_HPP_
