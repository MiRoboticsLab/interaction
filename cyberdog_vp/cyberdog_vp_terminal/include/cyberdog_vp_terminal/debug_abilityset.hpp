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

#ifndef CYBERDOG_VP_TERMINAL__DEBUG_ABILITYSET_HPP_
#define CYBERDOG_VP_TERMINAL__DEBUG_ABILITYSET_HPP_

#include <cyberdog_vp_abilityset/cyberdog.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <clocale>
#include <iostream>

#include "cyberdog_vp_terminal/common.hpp"

namespace cyberdog_visual_programming_terminal
{
/*! \file       debug_abilityset.hpp
    \brief      能力集调试器模块。
    \details    创建及初始化机器人能力集调试器，以便调试。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保各个接口的可用及稳定性。
    \note       确保接口的优化及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/
namespace VPA = cyberdog_visual_programming_abilityset;
class DebugAbilityset
{
public:
  DebugAbilityset();
  ~DebugAbilityset();
  void Launch();                                  /*!< 启动 */
  void Stop();                                    /*!< 停止 */

  bool Init(
    const std::shared_ptr<VPA::Cyberdog> &);      /*!< 初始化 */

private:
  bool SetCyberdog(bool);                         /*!< 设置铁蛋 */
  bool UpdateStatus();                            /*!< 更新铁蛋状态 */
  std::string KeyboardTeleop(const int);          /*!< 键盘遥控 */
  std::string TriggerFunction(
    const std::string);                           /*!< 触发功能 */
  std::vector<std::string> GetVector(
    const std::string &,
    char _delim = '\n');                          /*!< 获取向量 */
  void DrawParamsWin(WINDOW *, int);              /*!< 绘画参数窗口 */
  void InitParamsWins(WINDOW **, int);            /*!< 初始化参数窗口 */
  void InitStateWins(WINDOW **);                  /*!< 初始化状态窗口 */
  bool UpdataStateWins(WINDOW **);                /*!< 更新状态窗口 */
  uint GetX(uint, uint);                          /*!< 获取x坐标 */
  void WinUpdateFocus(
    WINDOW **, int, WINDOW *, const int,
    bool able = true);                            /*!< 窗口更新焦点 */

private:
  std::string logger_ {""};                       /*!< 日志名称 */
  std::string namespace_ {""};                    /*!< 名空间 */
  int default_window_ {1};                        /*!< 默认窗口 */
  int focus_window_ {2};                          /*!< 焦点窗口 */
  int default_column_ {3};                        /*!< 默认栏目 */
  int focus_column_ {4};                          /*!< 焦点栏目 */
  int warning_message_ {5};                       /*!< 警告消息 */
  int error_message_ {6};                         /*!< 错误消息 */
  int begin_lines_ {3};                           /*!< 起始行 */
  int begin_cols_ {1};                            /*!< 起始列 */
  int params_win_head_height_ {9};                /*!< 参数窗口头高度 */
  int incremental_x_ {4};                         /*!< 新窗口x坐标增量 */
  int incremental_y_ {3};                         /*!< 新窗口y坐标增量 */
  int function_win_height_ {11};                  /*!< 功能窗口高度 */
  int function_menu_height_ {5};                  /*!< 功能选项高度 */
  int function_win_width_ {0};                    /*!< 功能窗口宽度 */
  int state_win_width_ {0};                       /*!< 状态窗口宽度 */
  std::shared_ptr<std::thread>
  updata_thread_ptr_ {nullptr};                   /*!< 更新线程 */
  std::shared_ptr<VPA::Cyberdog>
  cyberdog_ptr_ {nullptr};                        /*!< 铁蛋能力集 */
  /*! 参数选项 */
  enum PARAMETERS
  {
    velocity = 0,                                 /*!< 速度 */
    centroid,                                     /*!< 质心 */
    fulcrum,                                      /*!< 支点 */
    angle,                                        /*!< 角度 */
    scale,                                        /*!< 尺度 */
    compensation,                                 /*!< 补偿 */
    x = 0,                                        /*!< X轴约束 */
    y,                                            /*!< Y轴约束 */
    z,                                            /*!< Z轴约束 */
    front_foot = 0,                               /*!< 前脚 */
    rears_foot,                                   /*!< 后脚 */
    duration,                                     /*!< 持续时间/耗时 */
    motion_frame = 0,                             /*!< 运动帧 */
    reserved,                                     /*!< 预留 */
  };
  /*! 参数约束:可调参数 */
  struct TunableParameters
  {
    std::string key;                              /*!< 键 */
    double value;                                 /*!< 值 */
    VPA::DefaultAndMaximum params;                /*!< 参数 */
  };

  /*! 参数窗口约束 */
  struct
  {
    // ┌───────────────────────────────────────────────────┐
    // │                       scale                       │
    // ├────────────┬────────────────────────────────┬─────┤
    // │            │              value             │     │
    // │   keys     ├──────────┬──────────┬──────────┤units│
    // │            │   min    │   now    │   max    │     │
    // ├────────────┼──────────┼──────────┼──────────┼─────┤
    // │ front-foot │   0.01   │   0.03   │   0.05   │  m  │
    // │ rears-foot │   0.01   │ 0.030000 │   0.05   │  m  │
    // │  duration  │    0     │     0    │   3600   │  s  │
    // ├────────────┼──────────┼──────────┼──────────┼─────┤
    // │    delt    │   0.1    │    10    │   100    │ per │
    // └────────────┴──────────┴──────────┴──────────┴─────┘
    std::vector<uint> length =
    {12, 10, 10, 10, 5};                          /*!< 长度 */
    std::vector<std::string> head = {             /*!< 头 */
      "value",
      "keys",
      "units",
      "min",
      "now",
      "max",
    };
    std::vector<std::pair<std::pair<std::string, int>,
      std::vector<TunableParameters>>> body = {   /*!< 身体 */
      {
        {"velocity", 0},
        {
          {"x-axis", VPA::motion_params_.x_velocity.default_value, VPA::motion_params_.x_velocity},
          {"y-axis", VPA::motion_params_.y_velocity.default_value, VPA::motion_params_.y_velocity},
          {"z-axis", VPA::motion_params_.z_velocity.default_value, VPA::motion_params_.z_velocity},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
      {
        {"centroid", 0},
        {
          {"x", VPA::motion_params_.centroid_x.default_value, VPA::motion_params_.centroid_x},
          {"y", VPA::motion_params_.centroid_y.default_value, VPA::motion_params_.centroid_y},
          {"z", VPA::motion_params_.centroid_z.default_value, VPA::motion_params_.centroid_z},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
      {
        {"fulcrum", 0},
        {
          {"x", VPA::motion_params_.fulcrum_x.default_value, VPA::motion_params_.fulcrum_x},
          {"y", VPA::motion_params_.fulcrum_y.default_value, VPA::motion_params_.fulcrum_y},
          {"z", VPA::motion_params_.fulcrum_z.default_value, VPA::motion_params_.fulcrum_z},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
      {
        {"angle", 0},
        {
          {"x-roll", VPA::motion_params_.roll.default_value, VPA::motion_params_.roll},
          {"y-pitch", VPA::motion_params_.pitch.default_value, VPA::motion_params_.pitch},
          {"z-yaw", VPA::motion_params_.yaw.default_value, VPA::motion_params_.yaw},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
      {
        {"scale", 0},
        {
          {"front-foot", VPA::motion_params_.front_leg_lift.default_value,
            VPA::motion_params_.front_leg_lift},
          {"rears-foot", VPA::motion_params_.back_leg_lift.default_value,
            VPA::motion_params_.back_leg_lift},
          {"duration", VPA::motion_params_.duration.default_value,
            VPA::motion_params_.duration},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
      {
        {"compensation", 0},
        {
          {"motion-frame", VPA::motion_params_.compensation_frame.default_value,
            VPA::motion_params_.compensation_frame},
          {"reserved-1", VPA::motion_params_.reserved.default_value,
            VPA::motion_params_.reserved},
          {"reserved-1", VPA::motion_params_.reserved.default_value,
            VPA::motion_params_.reserved},
          {"delt", VPA::motion_params_.delta.default_value, VPA::motion_params_.delta}
        }
      },
    };
  } params_window;                                /*!< [类型]约束参数 */
  /*! 功能选项 */
  enum FUNCTION
  {
    help = 0,                                     /*!< 帮助 */

    emergency_stop,                               /*!< 急停 */
    get_down,                                     /*!< 趴下 */
    resume_standing,                              /*!< 恢复站立 */
    back_flip,                                    /*!< 后空翻 */
    front_flip,                                   /*!< 前空翻 */
    bow,                                          /*!< 作揖 */
    roll_left,                                    /*!< 向左侧躺后恢复 */
    walk_the_dog,                                 /*!< 遛狗 */
    jump_stair,                                   /*!< 跳上台阶 */
    right_somersault,                             /*!< 右侧空翻 */
    left_somersault,                              /*!< 左侧空翻 */
    run_and_jump_front_flip,                      /*!< 跑跳前空翻 */
    jump3d_left90deg,                             /*!< 3D跳:左转90度 */
    jump3d_right90deg,                            /*!< 3D跳:右转90度 */
    jump3d_forward60cm,                           /*!< 3D跳:前跳60cm */
    jump3d_forward30cm,                           /*!< 3D跳:前跳30cm */
    jump3d_left20cm,                              /*!< 3D跳:左跳20cm */
    jump3d_right20cm,                             /*!< 3D跳:右跳20cm */
    jump3d_up30cm,                                /*!< 3D跳:向上30cm */
    jump3d_down_stair,                            /*!< 3D跳:跳下台阶 */
    roll_right,                                   /*!< 向右侧躺后恢复 */
    dance_collection,                             /*!< 舞蹈集合 */
    hold_left_hand,                               /*!< 握左手 */
    hold_right_hand,                              /*!< 握右手 */
    sit_down,                                     /*!< 坐下 */
    butt_circle,                                  /*!< 屁股画圆 */
    head_circle,                                  /*!< 头画圆 */
    stretch_the_body,                             /*!< 伸展身体 */
    shake_ass_left,                               /*!< 向左摇晃屁股 */
    shake_ass_right,                              /*!< 向右摇晃屁股 */
    shake_ass_from_side_to_side,                  /*!< 左右摇晃屁股 */
    ballet,                                       /*!< 芭蕾舞 */
    space_walk,                                   /*!< 太空步 */
    front_leg_jumping,                            /*!< 前腿开合跳 */
    hind_leg_jumping,                             /*!< 后腿开合跳 */
    lift_the_left_leg_and_nod,                    /*!< 左腿抬起并点头 */
    lift_the_right_leg_and_nod,                   /*!< 右腿抬起并点头 */
    left_front_right_back_legs_apart,             /*!< 左前右后岔开腿 */
    right_front_left_back_legs_apart,             /*!< 右前左后岔开腿 */
    walk_nodding,                                 /*!< 走路点头 */
    walking_with_divergence_and_adduction_alternately,
    /*!< 岔开内收交替走路 */
    nodding_in_place,                             /*!< 原地踏步点头 */
    front_legs_jump_back_and_forth,               /*!< 前腿前后跳 */
    hind_legs_jump_back_and_forth,                /*!< 后腿前后跳 */
    alternately_front_leg_lift,                   /*!< 前腿交替抬起 */
    alternately_hind_leg_lift,                    /*!< 后腿交替抬起 */
    jump_collection,                              /*!< 跳跃合集 */
    stretching_left_and_right,                    /*!< 左右伸腿踏步 */
    jump_forward_and_backward,                    /*!< 前后摆腿跳跃 */
    step_left_and_right,                          /*!< 左右摆腿踏步 */
    right_leg_back_and_forth_stepping,            /*!< 右腿前后踏步 */
    left_leg_back_and_forth_stepping,             /*!< 左腿前后踏步 */
    bow_to_each_other,                            /*!< 作揖比心 */

    absolute_force_control_attitude,              /*!< 绝对力控姿态（绝对姿态） */
    relatively_force_control_attitude,            /*!< 相对力控姿态 */
    absolute_position_control_attitude,           /*!< 绝对位控姿态（过渡站立） */
    relatively_position_control_attitude,         /*!< 相对位控姿态 */

    jump_back_and_forth,                          /*!< 前后跳 */
    small_jump_walking,                           /*!< 小跳行走 */
    trot_walking,                                 /*!< 小跑行走 */
    automatic_frequency_conversion_walking,       /*!< 自动变频行走 */
    run_fast_walking,                             /*!< 快跑行走 */

    Led_set_play,                                 /*!< LED灯/设置效果 */

    Audio_set_play,                               /*!< 语音模块/设置效果 */
  } walking_type;                                 /*!< 运动类型 */

  /*! 功能窗口约束 */
  struct
  {
    // ┌──────────────────────────────────────────────────────────┐
    // │                         function                         │
    // ├──────────────────────────────────────────────────────────┤
    // │$ xxx1  : 012345678901234567890123456789012345678901234567│
    // │  xxx2  xxx2 : xxxx                                       │
    // │  xxx3  xxx3 : xxxx                                       │
    // │  xxx4  xxx4 : xxxx                                       │
    // │  xxx5  xxx5 : xxxx                                       │
    // │                                                          │
    // └──────────────────────────────────────────────────────────┘
    std::string head = "function";                /*!< 头 */

    // body 个数必须为偶数
    std::vector<std::pair<std::string,
      std::pair<std::string, std::string>>> body =
    {                                             /*!< 身体 */
      {"base", {"help", "[Terminal] : show help documentation"}},

      {"motion", {"emergency_stop", "[Robot] : stand still or emergency stop"}},
      {"motion", {"get_down", "[Robot] : lie down"}},
      {"motion", {"resume_standing", "[Robot] : resume standing"}},
      {"motion", {"back_flip", "[Robot] : do a back flip"}},
      {"motion", {"front_flip", "[Robot] : do a front flip"}},
      {"motion", {"bow", "[Robot] : bow once"}},
      {"motion", {"roll_left", "[Robot] : roll to the left in a circle"}},
      {"motion", {"walk_the_dog", "[Robot] : enter dog walking mode"}},
      {"motion", {"jump_stair", "[Robot] : jump stair"}},
      {"motion", {"right_somersault", "[Robot] : right somersault"}},
      {"motion", {"left_somersault", "[Robot] : left somersault"}},
      {"motion", {"RAJ_front_flip", "[Robot] : run and jump front flip"}},
      {"motion", {"J3D_left90deg", "[Robot] : jump 3d left 90 deg"}},
      {"motion", {"J3D_right90deg", "[Robot] : jump 3d right 90 deg"}},
      {"motion", {"J3D_forward60cm", "[Robot] : jump 3d forward 60 cm"}},
      {"motion", {"J3D_forward30cm", "[Robot] : jump 3d forward 30 cm"}},
      {"motion", {"J3D_left20cm", "[Robot] : jump 3d left 20 cm"}},
      {"motion", {"J3D_right20cm", "[Robot] : jump 3d right 20 cm"}},
      {"motion", {"J3D_up30cm", "[Robot] : jump 3d up 30 cm"}},
      {"motion", {"J3D_down_stair", "[Robot] : jump 3d down stair"}},
      {"motion", {"roll_right", "[Robot] : roll right"}},
      {"motion", {"dance_collection", "[Robot] : dance collection"}},
      {"motion", {"hold_left_hand", "[Robot] : hold left hand"}},
      {"motion", {"hold_right_hand", "[Robot] : hold right hand"}},
      {"motion", {"sit_down", "[Robot] : sit down"}},
      {"motion", {"butt_circle", "[Robot] : butt circle"}},
      {"motion", {"head_circle", "[Robot] : head circle"}},
      {"motion", {"stretch_the_body", "[Robot] : stretch the body"}},
      {"motion", {"shake_ass_left", "[Robot] : shake ass left"}},
      {"motion", {"shake_ass_right", "[Robot] : shake ass right"}},
      {"motion", {"shake_ass", "[Robot] : shake ass from side to side"}},
      {"motion", {"ballet", "[Robot] : ballet"}},
      {"motion", {"space_walk", "[Robot] : space walk"}},
      {"motion", {"LJ_front", "[Robot] : front leg jumping"}},
      {"motion", {"LJ_hind", "[Robot] : hind leg jumping"}},
      {"motion", {"LLAN_left", "[Robot] : lift the left leg and nod"}},
      {"motion", {"LLAN_right", "[Robot] : lift the right leg and nod"}},
      {"motion", {"LFRB_legs_apart", "[Robot] : left front right back legs apart"}},
      {"motion", {"RFLB_legs_apart", "[Robot] : right front left back legs apart"}},
      {"motion", {"walk_nodding", "[Robot] : walk nodding"}},
      {"motion", {"walk_DAAA", "[Robot] : walking with divergence and adduction alternately"}},
      {"motion", {"nodding_in_place", "[Robot] : nodding in place"}},
      {"motion", {"JBAF_front_legs", "[Robot] : front legs jump back and forth"}},
      {"motion", {"JBAF_hind_legs", "[Robot] : hind legs jump back and forth"}},
      {"motion", {"ALL_front", "[Robot] : alternately front leg lift"}},
      {"motion", {"ALL_hind", "[Robot] : alternately hind leg lift"}},
      {"motion", {"jump_collection", "[Robot] : jump collection"}},
      {"motion", {"stretching_LAR", "[Robot] : stretching_left_and_right"}},
      {"motion", {"jump_FAB", "[Robot] : jump_forward_and_backward"}},
      {"motion", {"step_LAR", "[Robot] : step_left_and_right"}},
      {"motion", {"BAFS_right", "[Robot] : right_leg_back_and_forth_stepping"}},
      {"motion", {"BAFS_left", "[Robot] : left_leg_back_and_forth_stepping"}},
      {"motion", {"bow_to_each", "[Robot] : bow_to_each_other"}},
      {"motion", {"AFC_attitude", "[Robot] : absolute force control attitude"}},
      {"motion", {"RFC_attitude", "[Robot] : relatively force control attitude"}},
      {"motion", {"APC_attitude", "[Robot] : absolute position control attitude"}},
      {"motion", {"RPC_attitude", "[Robot] : relatively position control attitude"}},

      {"motion", {"jump_BAF", "[Teleop]: jump back and forth with keyboard"}},
      {"motion", {"SJ_walking", "[Teleop]: small jump walking with keyboard"}},
      {"motion", {"trot_walking", "[Teleop]: Trot walking with keyboard"}},
      {"motion", {"AFC_walking", "[Teleop]: automatic frequency conversion walking with keyboard"}},
      {"motion", {"RF_walking", "[Teleop]: run fast walking with keyboard"}},

      {"led", {"led_play", "[Robot] : control led light effect"}},

      // {"audio",   {"audio_play",      "[Robot] : Play voice or audio files"}},
    };
    std::string flag = ">> ";                     /*!< 请求标头 */
    std::string request;                          /*!< 请求 */
  } function_window;                              /*!< [类型]约束参数 */
  /*! 功能选项 */
  enum STATES
  {
    version = 0,                                  /*!< 版本 */
    wifi_ssid,                                    /*!< wifi名称 */
    wifi_signal,                                  /*!< wifi信号强度 */
    robot_ip,                                     /*!< 机器人IP */
    provider_ip,                                  /*!< 终端IP */
    electricity,                                  /*!< 电量 */
    temperature,                                  /*!< 温度 */
    voltage,                                      /*!< 电压 */
    current,                                      /*!< 电流 */
    audio,                                        /*!< 语音模块 */
    led,                                          /*!< led模块 */
    touch,                                        /*!< 触摸板模块 */
    gps,                                          /*!< gps模块 */
    lidar,                                        /*!< 雷达模块 */
    tof,                                          /*!< 激光测距模块 */
    ultrasonic,                                   /*!< 超声波模块 */
    imu,                                          /*!< 惯导 */
    odometer,                                     /*!< 里程计 */
  };
  /*! 状态窗口约束 */
  struct StateWindowMeta
  {
    std::string key;                              /*!< 键 */
    std::string value;                            /*!< 值 */
    bool update;                                  /*!< 是否更新 */
    int index;                                    /*!< 当前开始下标 */
  };
  /*! 状态窗口约束 */
  struct
  {
    // ┌───────────────────────────┐
    // │        robot state        │
    // ├───────────┬───────────────┤
    // │ robot-ip  │     null      │
    // ├───────────┼───────────────┤
    // │ wifi-ssid │     null      │
    // ├───────────┼───────────────┤
    // │wifi-signal│       0       │
    // ├───────────┼───────────────┤
    // │electricity│       0       │
    // ├───────────┼───────────────┤
    // │  voltage  │       0       │
    // ├───────────┼───────────────┤
    // │  current  │       0       │
    // ├───────────┼───────────────┤
    // │   audio   │     False     │
    // ├───────────┼───────────────┤
    // │    led    │     False     │
    // ├───────────┼───────────────┤
    // │   touch   │     False     │
    // ├───────────┼───────────────┤
    // │    gps    │     False     │
    // ├───────────┼───────────────┤
    // │   lidar   │     False     │
    // ├───────────┼───────────────┤
    // │    tof    │     False     │
    // ├───────────┼───────────────┤
    // │ultrasonic │     False     │
    // └───────────┴───────────────┘
    std::string head = "robot state";             /*!< 头 */
    std::vector<uint> length = {15, 15};          /*!< 长度 */
    std::vector<StateWindowMeta> body = {         /*!< 身体 */
      {"version", "v-2.0", true, 0},
      {"wifi-ssid", "", true, 0},
      {"wifi-signal", "0", true, 0},
      {"robot-ip", "", true, 0},
      {"provider-ip", "", true, 0},
      {"electricity(%)", "0", true, 0},
      {"temperature(*C)", "0", true, 0},
      {"voltage(mV)", "0", true, 0},
      {"current(mA)", "0", true, 0},
      {"audio", "", true, 0},
      {"led", "", true, 0},
      {"touch", "", true, 0},
      {"gps", "", true, 0},
      {"lidar", "", true, 0},
      {"tof", "", true, 0},
      {"ultrasonic", "", true, 0},
      {"imu", "", true, 0},
      {"odometer", "", true, 0},
    };
  } state_window;                                 /*!< [类型]约束参数 */

  /*! 状态窗口约束 */
  struct
  {
    std::vector<std::string> help_info = {        /*!< 帮助文档 */
      " 1.<global window>:_______________________________________________",
      "   [esc]: Exit the program.                                       |     +---------------+",
      "   [tab]: Activating and switching parameter windows.             +---> | motion teleop |",
      "   [space]: Activating the function window.                             +-------+-------+",
      "   [up]/[down]: Move focus up or down(supports looping).          +---> |  change gait  |",
      " 2.<function window>:_____________________________________________|     +-------+-------+",
      "   [left]: Release selected option.                                     |classic|special|",
      "   [right]: Check the current option(check to release).                 +-------+-------+",
      "   [pg-up]/[pg-down] : Page up or down on the options bar.        +---> | q w e | Q W E |",
      "   [enter]: Activate the currently selected function item.        |     |  \\|/  |  \\|/  |",
      " 3.<parameter window>:____________________________________________|     | z x c | Z X C |",
      "   [left]/[right]: Increase or decrease the current parameter.          |  /|\\  |  /|\\  |",
      "   * Non-Delt: The magnitude of change is Delt% of the maximum value.   | z x c | Z X C |",
      "   * Delt: The magnitude of change is 0.1.                              +-------+-------+",
    };
    std::vector<std::string> cyberdog_info = {    /*!< 铁蛋信息 */
      "            _                 _                     _               ",
      "           (_) _             | |                   | |              ",
      " _ __ ___   _ (_) ___  _   _ | |__    ___  _ __  __| |  ___    __ _ ",
      "| '_ ` _ \\ | |   / __|| | | || '_ \\  / _ \\| '__|/ _` | / _ \\  / _` |",
      "| | | | | || | _| (__ | |_| || |_) ||  __/| |  | (_| || (_) || (_| |",
      "|_| |_| |_||_|(_)\\___| \\__, ||_.__/  \\___||_|   \\__,_| \\___/  \\__, |",
      "                        __/ |                                  __/ |",
      "                       |___/           _ _                    |___/ ",
      "                                      | | |                           ",
      "                  __ _  ___   ___   __| | |__  _   _  ___             ",
      "                 / _` |/ _ \\ / _ \\ / _` | '_ \\| | | |/ _ \\            ",
      "                | (_| | (_) | (_) | (_| | |_) | |_| |  __/  _   _   _ ",
      "                 \\__, |\\___/ \\___/ \\__,_|_.__/ \\__, |\\___| (_) (_) (_)",
      "                  __/ |                         __/ |                 ",
      "                 |___/                         |___/                  ",
    };
  } base_window;                                  /*!< [类型]约束参数 */
};  // class DebugAbilityset
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__DEBUG_ABILITYSET_HPP_
