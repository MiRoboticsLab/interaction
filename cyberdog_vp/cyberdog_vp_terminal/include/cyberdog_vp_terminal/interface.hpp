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

#ifndef CYBERDOG_VP_TERMINAL__INTERFACE_HPP_
#define CYBERDOG_VP_TERMINAL__INTERFACE_HPP_

#include <string>
#include <vector>
#include <utility>

#include "cyberdog_vp_terminal/common.hpp"

namespace cyberdog_visual_programming_terminal
{
/*! \file       interface.hpp
    \brief      接口模块。
    \details    创建及初始化机器人接口模块，以便测试调用。
    \author     尚子涵
    \author     Shang Zihan
    \version    1.1.0.0
    \date       2023-02-06
    \pre        初始化铁蛋模块。
    \bug        确保接口文档的正确性。
    \note       确保接口的持续补充及同步。
    \copyright  [2023]-[2023] [Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.]
*/

class Interface
{
public:
  Interface();
  ~Interface();

  void Interface_(const std::string _fun)
  {
    this->HelpDocument(std::string(std::string(__FUNCTION__) + _fun));
  }

  struct Cyberdog
  {
public:
    Cyberdog() {}
    std::function<void(std::string)> show_info_;
    void Cyberdog_(std::string _fun)
    {
      this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
    }

    struct Network
    {
      Network() {}
      std::function<void(std::string)> show_info_;
      void Network_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } network_;

    struct Follow
    {
      Follow() {}
      std::function<void(std::string)> show_info_;
      void Follow_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } follow_;

    struct Motion
    {
      Motion() {}
      std::function<void(std::string)> show_info_;
      void Motion_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } motion_;

    struct Navigation
    {
      Navigation() {}
      std::function<void(std::string)> show_info_;
      void Navigation_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } navigation_;

    struct Task
    {
      Task() {}
      std::function<void(std::string)> show_info_;
      void Task_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } task_;

    struct Train
    {
      Train() {}
      std::function<void(std::string)> show_info_;
      void Train_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } train_;

    struct Personnel
    {
      Personnel() {}
      std::function<void(std::string)> show_info_;
      void Personnel_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } personnel_;

    struct Gesture
    {
      Gesture() {}
      std::function<void(std::string)> show_info_;
      void Gesture_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } gesture_;

    struct Skeleton
    {
      Skeleton() {}
      std::function<void(std::string)> show_info_;
      void Skeleton_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } skeleton_;

    struct Bms
    {
      Bms() {}
      std::function<void(std::string)> show_info_;
      void Bms_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } bms_;

    struct Led
    {
      Led() {}
      std::function<void(std::string)> show_info_;
      void Led_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } led_;

    struct Audio
    {
      Audio() {}
      std::function<void(std::string)> show_info_;
      void Audio_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } audio_;

    struct Touch
    {
      Touch() {}
      std::function<void(std::string)> show_info_;
      void Touch_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } touch_;

    struct Gps
    {
      Gps() {}
      std::function<void(std::string)> show_info_;
      void Gps_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } gps_;

    struct Tof
    {
      Tof() {}
      std::function<void(std::string)> show_info_;
      void Tof_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } tof_;

    struct Lidar
    {
      Lidar() {}
      std::function<void(std::string)> show_info_;
      void Lidar_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } lidar_;

    struct Ultrasonic
    {
      Ultrasonic() {}
      std::function<void(std::string)> show_info_;
      void Ultrasonic_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } ultrasonic_;

    struct Odometer
    {
      Odometer() {}
      std::function<void(std::string)> show_info_;
      void Odometer_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } odometer_;

    struct Imu
    {
      Imu() {}
      std::function<void(std::string)> show_info_;
      void Imu_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } imu_;
  } cyberdog_;

  struct Visual
  {
    Visual() {}
    std::function<void(std::string)> show_info_;
    void Visual_(const std::string _fun)
    {
      this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
    }

    struct Interface
    {
      Interface() {}
      std::function<void(std::string)> show_info_;
      void Interface_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } interface_;

    struct Debugger
    {
      Debugger() {}
      std::function<void(std::string)> show_info_;
      void Debugger_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }

      struct Abilityset
      {
        Abilityset() {}
        std::function<void(std::string)> show_info_;
        void Abilityset_(const std::string _fun)
        {
          this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
        }
      } abilityset_;

      struct Engine
      {
        Engine() {}
        std::function<void(std::string)> show_info_;
        void Engine_(const std::string _fun)
        {
          this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
        }
      } engine_;
    } debugger_;
  } visual_;

  struct Type
  {
    Type() {}
    std::function<void(std::string)> show_info_;
    void Type_(const std::string _fun)
    {
      this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
    }

    struct Enum
    {
      Enum() {}
      std::function<void(std::string)> show_info_;
      void Enum_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } enum_;

    struct Class
    {
      Class() {}
      std::function<void(std::string)> show_info_;
      void Class_(const std::string _fun)
      {
        this->show_info_(std::string(std::string(__FUNCTION__) + _fun));
      }
    } class_;
  } type_;

private:
  std::string logger_ {""};                       /*!< 日志名称 */
  std::string interface_pkg_dir_ {""};            /*!< params 包路径 */
  std::string interface_config_dir_ {""};         /*!< node 配置文件路径 */
  toml::value interface_toml_;                    /*!< 配置文件数据 */

  std::string interface_constraint {"constraint"};
  /*!< 接口约束 */
  std::string interface_describe {"describe"};    /*!< 接口描述 */
  std::string interface_demo {"demo"};            /*!< 接口举例 */
  std::string interface_version {"version"};      /*!< 接口版本 */
  std::string interface_help {"help"};            /*!< 接口帮助文档 */
  std::string interface_list {"list"};            /*!< 接口列表 */
  std::string interface_tree {"tree"};            /*!< 接口树 */
  std::string interface_state {"state"};          /*!< 状态 */
  std::string interface_data {"data"};            /*!< 数据 */
  std::string interface_type {"type"};            /*!< 类型 */

  std::string RESET {"\033[0m"};                  /*!< 关闭所有属性 */
  std::string HIGHLIGHT {"\33[1m"};               /*!< 高亮 */
  std::string Brightness_Halved {"\33[2m"};       /*!< 亮度减半 */
  std::string ITALICS {"\33[3m"};                 /*!< 斜体 */
  std::string UNDERLINE {"\33[4m"};               /*!< 下划线 */
  std::string TWINKLE {"\33[5m"};                 /*!< 闪烁 */
  std::string FLASH {"\33[6m"};                   /*!< 快闪 */
  std::string REFLEXIVITY {"\33[7m"};             /*!< 反显 */
  std::string LATENT {"\33[8m"};                  /*!< 消隐 */
  std::string NOTES {"\33[9m"};                   /*!< 中间一道横线 */

  std::string TYPEFACE_A {"\33[10m"};             /*!< 字体 A */
  std::string TYPEFACE_B {"\33[11m"};             /*!< 字体 B */
  std::string TYPEFACE_C {"\33[12m"};             /*!< 字体 C */
  std::string TYPEFACE_D {"\33[13m"};             /*!< 字体 D */
  std::string TYPEFACE_E {"\33[14m"};             /*!< 字体 E */
  std::string TYPEFACE_F {"\33[15m"};             /*!< 字体 F */
  std::string TYPEFACE_G {"\33[16m"};             /*!< 字体 G */
  std::string TYPEFACE_H {"\33[17m"};             /*!< 字体 H */
  std::string TYPEFACE_I {"\33[18m"};             /*!< 字体 I */
  std::string TYPEFACE_J {"\33[19m"};             /*!< 字体 J */

  std::string NO_HIGHLIGHT {"\33[21m"};           /*!< 不高亮 */
  std::string NO_Brightness_HALVED {"\33[22m"};   /*!< 不亮度减半 */
  std::string NO_ITALICS {"\33[23m"};             /*!< 不斜体 */
  std::string NO_UNDERLINE {"\33[24m"};           /*!< 不下划线 */
  std::string NO_TWINKLE {"\33[25m"};             /*!< 不闪烁 */
  std::string NO_FLASH {"\33[26m"};               /*!< 不快闪 */
  std::string NO_REFLEXIVITY {"\33[27m"};         /*!< 不反显 */
  std::string NO_LATENT {"\33[28m"};              /*!< 不消隐 */
  std::string NO_NOTES {"\33[29m"};               /*!< 不中间一道横线 */

  std::string ForeCOLOR_BLACK {"\033[30m"};       /*!< 黑色字 */
  std::string ForeCOLOR_RED {"\033[31m"};         /*!< 红色字 */
  std::string ForeCOLOR_GREEN {"\033[32m"};       /*!< 绿色字 */
  std::string ForeCOLOR_YELLOW {"\033[33m"};      /*!< 黄色字 */
  std::string ForeCOLOR_BLUE {"\033[34m"};        /*!< 蓝色字 */
  std::string ForeCOLOR_MAGENTA {"\033[35m"};     /*!< 紫色字 */
  std::string ForeCOLOR_CYAN {"\033[36m"};        /*!< 天蓝字 */
  std::string ForeCOLOR_WHITE {"\033[37m"};       /*!< 白色字 */

  std::string UNDERLINE_ON {"\033[38m"};          /*!< 打开下划线,设置默认前景色 */
  std::string UNDERLINE_OFF {"\033[39m"};         /*!< 关闭下划线,设置默认前景色 */

  std::string BACKCOLOR_BLACK {"\033[40m"};       /*!< 黑色背景 */
  std::string BACKCOLOR_RED {"\033[41m"};         /*!< 红色背景 */
  std::string BACKCOLOR_GREEN {"\033[42m"};       /*!< 绿色背景 */
  std::string BACKCOLOR_YELLOW {"\033[43m"};      /*!< 黄色背景 */
  std::string BACKCOLOR_BLUE {"\033[44m"};        /*!< 蓝色背景 */
  std::string BACKCOLOR_MAGENTA {"\033[45m"};     /*!< 紫色背景 */
  std::string BACKCOLOR_CYAN {"\033[46m"};        /*!< 孔雀蓝背景 */
  std::string BACKCOLOR_WHITE {"\033[47m"};       /*!< 白色背景 */
  std::string BACKCOLOR_UNKNOWN {"\033[48m"};     /*!< 未知黄色背景 */
  std::string BACKCOLOR_DEFAULT {"\033[49m"};     /*!< 默认色背景 */

  std::string CURSOR_UP_N_LINES {"\033[nA"};      /*!< 光标上移n行 */
  std::string CURSOR_DOWN_N_LINES {"\033[nB"};    /*!< 光标下移n行 */
  std::string CURSOR_RIGHT_N_LINES {"\033[nC"};   /*!< 光标右移n行 */
  std::string CURSOR_LEFT_N_LINE {"\033[nD"};     /*!< 光标左移n行 */
  std::string SET_CURSOR_POSITION {"\033[y;xH"};  /*!< 设置光标位置 */
  std::string CLEANING_SCREEN {"\033[2J"};        /*!< 清屏 */
  std::string CLEARS_CURSOR_LINE {"\033[K"};      /*!< 清除从光标到行尾的内容 */
  std::string SAVE_CURSOR_LOCATION {"\033[s"};    /*!< 保存光标位置 */
  std::string RESTORE_CURSOR_LOCATION{"\033[u"};  /*!< 恢复光标位置 */
  std::string HIDDEN_CURSOR {"\033[?25l"};        /*!< 隐藏光标 */
  std::string DISPLAY_CURSOR {"\033[?25h"};       /*!< 显示光标 */

  std::string SLASH {"\\"};                       /*!< 换行 */
  std::string NEW_LINE {"\n"};                    /*!< 换行 */
  std::string SPACE {" "};                        /*!< 空格 */
  std::string VERTICAL_FRAME {"┃"};               /*!< 纵边框 */
  std::string HORIZONTAL_FRAME {"━"};             /*!< 横边框 */
  std::string LEFT_UP_CORNER {"┏"};               /*!< 左上拐角 */
  std::string LEFT_DOWN_CORNER {"┗"};             /*!< 左下拐角 */
  std::string RIGHT_UP_CORNER {"┓"};              /*!< 右上拐角 */
  std::string RIGHT_DOWN_CORNER {"┛"};            /*!< 右下拐角 */

  std::string LEFT_JOINER {"┠"};                  /*!< 左连接符 */
  std::string MIDDLE_HORIZONTAL {"─"};            /*!< 中横连接符 */
  std::string RIGHT_JOINER {"┨"};                 /*!< 右连接符 */
  std::string SUPERIOR_JOINER {"┯"};              /*!< 上连接符 */
  std::string MIDDLE_VERTICAL {"│"};              /*!< 中纵连接符 */
  std::string DOWN_JOINER {"┷"};                  /*!< 下连接符 */
  std::string SUPERIOR_DOWN_JOINER {"┳"};         /*!< 上下连接符 */
  std::string DOWN_SUPERIOR_JOINER {"┻"};         /*!< 下上连接符 */
  std::string MIDDLE_JOINER_LEFT {"┫"};           /*!< 中左连接符 */
  std::string MIDDLE_JOINER_RIGHT {"┣"};          /*!< 中右连接符 */
  std::string LEFT_MARGIN {"  "};                 /*!< 左边距 */
  std::string LEFT_BRACKETS {"("};                /*!< 左括号 */
  std::string RIGHT_BRACKETS {")"};               /*!< 右括号 */
  std::string LIST_ARROW {"➤ "};                  /*!< 列表箭头 */
  std::string LIST_ROOT {"● "};                   /*!< 列表根 */
  std::string LIST_BIFURCATION {"◎ "};            /*!< 列表分叉 */

private:
  bool Init();                                    /*!< 初始化数据 */

private:
  using InterfacePrecursorName =
    std::vector<std::string>;                     /*!< 接口前驱名称 */
  using InterfacePrecursorTree =
    std::vector<bool>;                            /*!< 接口前驱树木 */
  using InterfaceMeta =
    std::pair<bool,
      std::pair<InterfacePrecursorName,
      std::string>>;                              /*!< <是否为父接口, <接口前驱, 接口名称>>> */
  using InterfaceList = std::vector<InterfaceMeta>;
  int GetSubSize(
    const std::string &,
    const std::string &);                         /*!< 获取子串 */
  std::string GetConstraint(
    const std::vector<std::string> &,
    const toml::value &);                         /*!< 获取约束 */
  std::string GetDescribe(const toml::value &);   /*!< 获取描述 */
  std::string GetDemo(const toml::value &);       /*!< 获取例子 */
  bool GetForm(
    const std::vector<std::string> &,
    const toml::value &, InterfaceList &);        /*!< 获取表单 */
  std::string GetList(
    const std::vector<std::string> &,
    const toml::value &);                         /*!< 获取列表 */
  std::string GetTree(
    const std::vector<std::string> &,
    const toml::value &);                         /*!< 获取树形 */
  void HelpDocument(std::string);                 /*!< 帮助文档 */
};  // class Interface
}  // namespace cyberdog_visual_programming_terminal
#endif  // CYBERDOG_VP_TERMINAL__INTERFACE_HPP_
