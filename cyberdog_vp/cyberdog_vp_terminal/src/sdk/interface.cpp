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

#include <string>
#include <vector>
#include <utility>

#include "cyberdog_vp_terminal/interface.hpp"

namespace cyberdog_visual_programming_terminal
{
Interface::Interface()
: interface_config_dir_("/config/interface.toml")
{
  try {
    this->logger_ = "[" + std::string(__FUNCTION__) + "]";
    DEBUG("%s Creating object...", this->logger_.c_str());
    this->Init();
  } catch (const std::exception & e) {
    DEBUG("%s Creating object failed, %s", this->logger_.c_str(), e.what());
  }
}

Interface::~Interface()
{
  DEBUG("%s Destroy object...", this->logger_.c_str());
}

bool Interface::Init()
{
  try {
    DEBUG("%s Initializing data...", this->logger_.c_str());

    this->interface_pkg_dir_ = ament_index_cpp::get_package_share_directory("cyberdog_vp");
    this->interface_config_dir_ = this->interface_pkg_dir_ + this->interface_config_dir_;

    if (access(this->interface_config_dir_.c_str(), F_OK)) {
      ERROR(
        "%s Interface config file does not exist, config file dir:\n%s",
        this->logger_.c_str(), this->interface_config_dir_.c_str());
      return false;
    }
    if (access(this->interface_config_dir_.c_str(), R_OK)) {
      ERROR(
        "%s Interface config file does not have read permissions, config file dir:\n%s",
        this->logger_.c_str(), this->interface_config_dir_.c_str());
      return false;
    }
    if (access(this->interface_config_dir_.c_str(), W_OK)) {
      ERROR(
        "%s Interface config file does not have write permissions, config file dir:\n%s",
        this->logger_.c_str(), this->interface_config_dir_.c_str());
      return false;
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(
        this->interface_config_dir_.c_str(), this->interface_toml_))
    {
      ERROR(
        "%s Interface config file is not in toml format, config file dir:\n%s",
        this->logger_.c_str(), this->interface_config_dir_.c_str());
      return false;
    }

    this->cyberdog_.show_info_ = std::bind(&Interface::Interface_, this, std::placeholders::_1);
    this->cyberdog_.network_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_,
      this->cyberdog_, std::placeholders::_1);
    this->cyberdog_.follow_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.motion_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.navigation_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_,
      this->cyberdog_, std::placeholders::_1);
    this->cyberdog_.task_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.train_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.personnel_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.gesture_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.skeleton_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.bms_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.led_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.audio_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.touch_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.gps_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.tof_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.lidar_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);
    this->cyberdog_.ultrasonic_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_,
      this->cyberdog_, std::placeholders::_1);
    this->cyberdog_.odometer_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_,
      this->cyberdog_, std::placeholders::_1);
    this->cyberdog_.imu_.show_info_ = std::bind(
      &Interface::Cyberdog::Cyberdog_, this->cyberdog_,
      std::placeholders::_1);

    this->visual_.show_info_ = std::bind(&Interface::Interface_, this, std::placeholders::_1);
    this->visual_.interface_.show_info_ = std::bind(
      &Interface::Visual::Visual_, this->visual_,
      std::placeholders::_1);
    this->visual_.debugger_.show_info_ = std::bind(
      &Interface::Visual::Visual_, this->visual_,
      std::placeholders::_1);
    this->visual_.debugger_.abilityset_.show_info_ = std::bind(
      &Interface::Visual::Debugger::Debugger_, this->visual_.debugger_,
      std::placeholders::_1);
    this->visual_.debugger_.engine_.show_info_ = std::bind(
      &Interface::Visual::Debugger::Debugger_, this->visual_.debugger_,
      std::placeholders::_1);

    this->type_.show_info_ = std::bind(&Interface::Interface_, this, std::placeholders::_1);
    this->type_.enum_.show_info_ = std::bind(
      &Interface::Type::Type_, this->type_,
      std::placeholders::_1);
    this->type_.class_.show_info_ = std::bind(
      &Interface::Type::Type_, this->type_,
      std::placeholders::_1);
  } catch (const std::exception & e) {
    ERROR("%s Init data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

int Interface::GetSubSize(const std::string & str, const std::string & sub)
{
  int num = 0;
  size_t len = sub.length();
  if (len == 0) {len = 1;}
  for (size_t i = 0; (i = str.find(sub, i)) != std::string::npos; num++, i += len) {
  }
  return num;
}

std::string Interface::GetConstraint(
  const std::vector<std::string> & _interface,
  const toml::value & _root)
{
  auto get_interface_size = [&]() -> size_t {
      size_t ret = _interface.size();
      for (auto meta : _interface) {
        ret += meta.size();
      }
      return ret;
    };
  using ConstraintsType = std::vector<
    std::pair<
      std::pair<std::string, std::string>,
      std::string>>;  // [[type, key], demo_value]
  ConstraintsType invalids = {{{"type", "key"}, "demo_value"}};
  ConstraintsType constraints = toml::find_or(_root, interface_constraint, invalids);
  if (!constraints.empty() &&
    (constraints.front().first.first == invalids.front().first.first) &&
    (constraints.front().first.second == invalids.front().first.second) &&
    (constraints.front().second == invalids.front().second))
  {
    WARN(
      "GetConstraint() is invalid(not found), "
      "please contact the developer to update the protocol documentation.");
    return "";
  }
  size_t formats_max, example_max, now_size;
  formats_max = example_max = get_interface_size();
  for (ConstraintsType::iterator constraint_ptr = constraints.begin();
    constraint_ptr < constraints.end(); constraint_ptr++)
  {
    now_size = constraint_ptr->first.first.size() + constraint_ptr->first.second.size() + 2;
    if (now_size > formats_max) {
      formats_max = now_size;
    }
    now_size = constraint_ptr->second.size();
    if (now_size > example_max) {
      example_max = now_size;
    }
  }
  int width = 1;
  for (size_t i = constraints.size(); i != 0; width++) {
    i /= 10;
  }
  formats_max += 7 + width * 2;
  example_max += 5 + width * 2;
  std::string formats_horizontal_frame = "", example_horizontal_frame = "";
  for (size_t i = 0; i < formats_max; i++) {
    formats_horizontal_frame += HORIZONTAL_FRAME;
  }
  for (size_t i = 0; i < example_max; i++) {
    example_horizontal_frame += HORIZONTAL_FRAME;
  }
  std::ostringstream ret;
  ret << HIGHLIGHT << ForeCOLOR_RED << "★ " << ForeCOLOR_CYAN << "接口约束:" << RESET << NEW_LINE;
  ret << LEFT_MARGIN << LEFT_UP_CORNER << formats_horizontal_frame << SUPERIOR_JOINER <<
    example_horizontal_frame << RIGHT_UP_CORNER << NEW_LINE;
  std::string line_number_style = BACKCOLOR_YELLOW + ForeCOLOR_BLACK,
    formats_style = HIGHLIGHT + BACKCOLOR_WHITE + NO_ITALICS,
    example_style = HIGHLIGHT + BACKCOLOR_BLACK + ITALICS;
  auto add_protocol_interface_name = [&](std::string & style, size_t max_size) {
      size_t space_size = max_size;
      ret << line_number_style << std::setw(width) << 0 << style << LEFT_MARGIN;
      space_size -= (width + LEFT_MARGIN.size());
      size_t count = _interface.size() - 1;
      for (size_t i = 0; i < count; i++) {
        switch (i % 4) {
          case 0: ret << ForeCOLOR_GREEN; break;
          case 1: ret << ForeCOLOR_YELLOW; break;
          case 2: ret << ForeCOLOR_BLUE; break;
          case 3: ret << ForeCOLOR_MAGENTA; break;
          default:
            break;
        }
        ret << _interface.at(i) << ForeCOLOR_RED << '.';
        space_size -= (_interface.at(i).size() + 1);
      }
      ret << ForeCOLOR_CYAN << _interface.at(count) << ForeCOLOR_MAGENTA << LEFT_BRACKETS;
      space_size -= (_interface.at(count).size() + LEFT_BRACKETS.size());
      if (constraints.empty()) {
        ret << RIGHT_BRACKETS;
        space_size -= (RIGHT_BRACKETS.size());
      }
      for (size_t i = 0; i < space_size; i++) {
        ret << SPACE;
      }
    };
  ret << LEFT_MARGIN << VERTICAL_FRAME;
  add_protocol_interface_name(formats_style, formats_max);
  ret << RESET << MIDDLE_VERTICAL;
  add_protocol_interface_name(example_style, example_max);
  ret << RESET << VERTICAL_FRAME << NEW_LINE;
  int index = 1;
  for (ConstraintsType::iterator constraint_ptr = constraints.begin();
    constraint_ptr < constraints.end(); constraint_ptr++, index++)
  {
    bool is_last = ((constraint_ptr + 1) == constraints.end());
    size_t space_size;
    ret << LEFT_MARGIN << VERTICAL_FRAME;
    ret << line_number_style << std::setw(width) << index << formats_style << LEFT_MARGIN <<
      LEFT_MARGIN << ForeCOLOR_CYAN << constraint_ptr->first.first << ForeCOLOR_YELLOW << " " <<
      constraint_ptr->first.second << ForeCOLOR_RED;
    space_size = formats_max - width - LEFT_MARGIN.size() * 2 - constraint_ptr->first.first.size() -
      constraint_ptr->first.second.size() - 1;
    if (!is_last) {
      ret << ForeCOLOR_RED << ",";
      space_size -= 1;
    }
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << MIDDLE_VERTICAL;
    ret << line_number_style << std::setw(width) << index << example_style << LEFT_MARGIN <<
      LEFT_MARGIN << ForeCOLOR_YELLOW << constraint_ptr->second;
    space_size = example_max - width - LEFT_MARGIN.size() * 2 - constraint_ptr->second.size();
    if (!is_last) {
      ret << ForeCOLOR_RED << ",";
      space_size -= 1;
    }
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << VERTICAL_FRAME << NEW_LINE;
  }
  if (!constraints.empty()) {
    size_t space_size;
    ret << LEFT_MARGIN << VERTICAL_FRAME;
    ret << line_number_style << std::setw(width) << index << formats_style << LEFT_MARGIN <<
      LEFT_MARGIN << ForeCOLOR_MAGENTA << RIGHT_BRACKETS;
    space_size = formats_max - width - LEFT_MARGIN.size() * 2 - RIGHT_BRACKETS.size();
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << MIDDLE_VERTICAL;
    ret << line_number_style << std::setw(width) << index << example_style << LEFT_MARGIN <<
      LEFT_MARGIN << ForeCOLOR_MAGENTA << RIGHT_BRACKETS;
    space_size = example_max - width - LEFT_MARGIN.size() * 2 - RIGHT_BRACKETS.size();
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << VERTICAL_FRAME << NEW_LINE;
  }
  ret << LEFT_MARGIN << LEFT_DOWN_CORNER << formats_horizontal_frame << DOWN_JOINER <<
    example_horizontal_frame << RIGHT_DOWN_CORNER << NEW_LINE;
  return ret.str();
}

std::string Interface::GetDescribe(const toml::value & _root)
{
  using DescribeType = std::vector<std::string>;
  DescribeType invalids = {"invalids"};
  DescribeType describes = toml::find_or(_root, interface_describe, invalids);
  if (!describes.empty() &&
    (describes.front() == invalids.front()))
  {
    WARN(
      "GetDescribe() is invalid(not found), "
      "please contact the developer to update the protocol documentation.");
    return "";
  }
  std::ostringstream ret;
  ret << HIGHLIGHT << ForeCOLOR_RED << "★ " << ForeCOLOR_CYAN << "协议说明:" << RESET << NEW_LINE;
  ret << ForeCOLOR_WHITE;
  for (std::vector<std::string>::iterator describe_ptr = describes.begin();
    describe_ptr != describes.end();
    describe_ptr++)
  {
    ret << LEFT_MARGIN << *describe_ptr << NEW_LINE;
  }
  ret << RESET;
  return ret.str();
}

std::string Interface::GetDemo(const toml::value & _root)
{
  using DemoType = std::vector<std::string>;
  DemoType invalids = {"invalids"};
  DemoType demos = toml::find_or(_root, interface_demo, invalids);
  if (!demos.empty() &&
    (demos.front() == invalids.front()))
  {
    WARN(
      "GetDemo() is invalid(not found), "
      "please contact the developer to update the protocol documentation.");
    return "";
  }
  int width = 1;
  for (size_t i = demos.size(); i != 0; width++) {
    i /= 10;
  }
  std::string line_number_style = BACKCOLOR_YELLOW + ForeCOLOR_BLACK,
    data_style = HIGHLIGHT + BACKCOLOR_BLACK + ITALICS;
  size_t protocol_max = 0;
  for (DemoType::iterator demo_ptr = demos.begin();
    demo_ptr != demos.end();
    ++demo_ptr)
  {
    if (demo_ptr->size() > protocol_max) {
      protocol_max = demo_ptr->size();
    }
  }
  std::string horizontal_frame = "";
  protocol_max += 5 + width;
  for (size_t i = 0; i < protocol_max; i++) {
    horizontal_frame += HORIZONTAL_FRAME;
  }
  std::ostringstream ret;
  ret << HIGHLIGHT << ForeCOLOR_RED << "★ " << ForeCOLOR_CYAN << "协议示例:" << RESET << NEW_LINE;
  ret << LEFT_MARGIN << LEFT_UP_CORNER << horizontal_frame << RIGHT_UP_CORNER << NEW_LINE;
  std::string demo;
  int index = 0;
  for (DemoType::iterator demo_ptr = demos.begin();
    demo_ptr != demos.end();
    ++demo_ptr, index++)
  {
    demo = *demo_ptr;
    ret << LEFT_MARGIN << VERTICAL_FRAME << line_number_style << std::setw(width) << index <<
      data_style << ForeCOLOR_CYAN << LEFT_MARGIN << demo;
    size_t space_size = protocol_max - width - LEFT_MARGIN.size() - demo.size() + GetSubSize(
      demo,
      SLASH);
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << VERTICAL_FRAME << NEW_LINE;
  }
  ret << LEFT_MARGIN << LEFT_DOWN_CORNER << horizontal_frame << RIGHT_DOWN_CORNER << NEW_LINE;
  return ret.str();
}

bool Interface::GetForm(
  const std::vector<std::string> & _interface, const toml::value & _root,
  InterfaceList & _module_list)
{
  try {
    // auto cout_module_list = [&](std::string _msg) {
    //     std::cout << "<------------" << _msg << "---------->" << std::endl;
    //     for (auto it1_Ptr = _module_list.rbegin(); it1_Ptr != _module_list.rend(); ++it1_Ptr) {
    //       std::cout << "[" << it1_Ptr->first << "] ";
    //       for (auto it2_Ptr = it1_Ptr->second.first.begin();
    //         it2_Ptr != it1_Ptr->second.first.end(); ++it2_Ptr)
    //       {
    //         std::cout << *it2_Ptr << ".";
    //       }
    //       std::cout << it1_Ptr->second.second << std::endl;
    //     }
    //   };
    std::function<bool(
        const toml::value &,
        const std::string &,
        const InterfacePrecursorName &)> get_list = [&](
      const toml::value & _tar_root,
      const std::string & _tar_key,
      const InterfacePrecursorName & _precursor) -> bool {
        if (!_tar_root.is_table()) {return false;}
        InterfacePrecursorName now_precursor(_precursor);
        if (!_tar_key.empty()) {
          now_precursor.push_back(_tar_key);
        }
        bool is_interface = false;
        for (const auto & [now_key, now_root] : _tar_root.as_table()) {
          if ((now_key == interface_version) ||
            (now_key == interface_help) ||
            (now_key == interface_constraint) ||
            (now_key == interface_describe) ||
            (now_key == interface_demo))
          {
            continue;
          }
          is_interface = get_list(now_root, now_key, now_precursor);
        }
        if (!_tar_key.empty()) {
          _module_list.push_back(InterfaceMeta{is_interface, {_precursor, _tar_key}});
        }
        return true;
      };
    InterfacePrecursorName module_precursor;
    for (auto meta : _interface) {
      _module_list.push_back(InterfaceMeta{true, {module_precursor, meta}});
      module_precursor.push_back(meta);
    }
    if (!get_list(_root, "", module_precursor)) {
      ERROR("%s Get list data failed at get_list().", this->logger_.c_str());
      return false;
    }
    // cout_module_list("old module list");
    std::sort(
      _module_list.begin(), _module_list.end(),
      [](const InterfaceMeta a, const InterfaceMeta b)
      {
        auto get_interface_str = [&](const InterfaceMeta & _meta) -> std::string {
          std::string str;
          for (auto head_Ptr = _meta.second.first.begin(); head_Ptr != _meta.second.first.end();
          ++head_Ptr)
          {
            str += *head_Ptr;
          }
          str += _meta.second.second;
          return str;
        };
        return get_interface_str(a) > get_interface_str(b);
      });
    // cout_module_list("new module list");
    return true;
  } catch (const std::exception & e) {
    ERROR("%s Get list data failed: %s", this->logger_.c_str(), e.what());
  }
  return false;
}

std::string Interface::GetList(
  const std::vector<std::string> & _interface,
  const toml::value & _root)
{
  InterfaceList module_list;
  if (!this->GetForm(_interface, _root, module_list)) {
    return "";
  }
  size_t protocol_max = 0, interface_now = 0;
  for (auto it1_Ptr = module_list.rbegin(); it1_Ptr != module_list.rend(); ++it1_Ptr) {
    interface_now = it1_Ptr->second.second.size();
    for (auto it2_Ptr = it1_Ptr->second.first.begin(); it2_Ptr != it1_Ptr->second.first.end();
      ++it2_Ptr)
    {
      interface_now += it2_Ptr->size() + 1;
    }
    if (interface_now > protocol_max) {
      protocol_max = interface_now;
    }
  }
  int width = 3, interface_index = 0;
  protocol_max += 5 + width;
  std::string horizontal_frame = "";
  for (size_t i = 0; i < protocol_max; i++) {
    horizontal_frame += HORIZONTAL_FRAME;
  }
  std::ostringstream ret;
  ret << HIGHLIGHT << ForeCOLOR_RED << "★ " << ForeCOLOR_CYAN << "协议列表:" << RESET << NEW_LINE;
  ret << LEFT_MARGIN << LEFT_UP_CORNER << horizontal_frame << RIGHT_UP_CORNER << NEW_LINE;
  std::string line_number_style = BACKCOLOR_YELLOW + ForeCOLOR_BLACK,
    data_style = HIGHLIGHT + BACKCOLOR_BLACK + ITALICS;
  for (auto it1_Ptr = module_list.rbegin(); it1_Ptr != module_list.rend(); ++it1_Ptr) {
    if (it1_Ptr->first) {continue;}
    ret << LEFT_MARGIN << VERTICAL_FRAME << line_number_style << std::setw(width) <<
      ++interface_index << data_style << LEFT_MARGIN;
    size_t space_size = protocol_max - width - LEFT_MARGIN.size() - it1_Ptr->second.second.size();
    for (auto it2_Ptr = it1_Ptr->second.first.begin(); it2_Ptr != it1_Ptr->second.first.end();
      ++it2_Ptr)
    {
      ret << ForeCOLOR_MAGENTA << *it2_Ptr << ForeCOLOR_BLUE << ".";
      space_size -= (it2_Ptr->size() + 1);
    }
    if ((it1_Ptr->second.second == "state") ||
      (it1_Ptr->second.second == "data") ||
      (it1_Ptr->second.second == "params"))
    {
      ret << ForeCOLOR_WHITE << it1_Ptr->second.second;
    } else {
      ret << ForeCOLOR_GREEN << it1_Ptr->second.second << ForeCOLOR_CYAN << LEFT_BRACKETS <<
        RIGHT_BRACKETS;
      space_size -= 2;
    }
    for (size_t i = 0; i < space_size; i++) {
      ret << SPACE;
    }
    ret << RESET << VERTICAL_FRAME << NEW_LINE;
  }
  ret << LEFT_MARGIN << LEFT_DOWN_CORNER << horizontal_frame << RIGHT_DOWN_CORNER << NEW_LINE;
  return ret.str();
}

std::string Interface::GetTree(
  const std::vector<std::string> & _interface,
  const toml::value & _root)
{
  InterfaceList module_list;
  if (!this->GetForm(_interface, _root, module_list)) {
    return "";
  }
  size_t protocol_max = 0, interface_now = 0;
  for (auto it1_Ptr = module_list.rbegin(); it1_Ptr != module_list.rend(); ++it1_Ptr) {
    interface_now = it1_Ptr->second.second.size();
    for (auto it2_Ptr = it1_Ptr->second.first.begin(); it2_Ptr != it1_Ptr->second.first.end();
      ++it2_Ptr)
    {
      interface_now += it2_Ptr->size() + 1;
    }
    if (interface_now > protocol_max) {
      protocol_max = interface_now;
    }
  }
  int width = 3, interface_index = 0;
  protocol_max += width;
  std::string horizontal_frame = "";
  for (size_t i = 0; i < protocol_max; i++) {
    horizontal_frame += HORIZONTAL_FRAME;
  }
  std::string line_left_style = RESET + HIGHLIGHT + ForeCOLOR_WHITE + BACKCOLOR_YELLOW;
  std::string line_right_style = RESET + HIGHLIGHT + ForeCOLOR_WHITE + BACKCOLOR_BLACK;
  std::string line_number_style = RESET + ForeCOLOR_BLACK + BACKCOLOR_YELLOW;
  std::string list_tree_style = RESET + HIGHLIGHT + ForeCOLOR_BLUE + BACKCOLOR_BLACK;
  std::string list_module_style = RESET + HIGHLIGHT + ForeCOLOR_MAGENTA + ITALICS + BACKCOLOR_BLACK;
  std::string list_function_style = RESET + HIGHLIGHT + ForeCOLOR_GREEN + ITALICS + BACKCOLOR_BLACK;
  std::string list_variable_style = RESET + HIGHLIGHT + ForeCOLOR_WHITE + ITALICS + BACKCOLOR_BLACK;
  std::ostringstream ret;
  ret << HIGHLIGHT << ForeCOLOR_RED << "★ " << ForeCOLOR_CYAN << "协议树:" << RESET << NEW_LINE;
  ret << LEFT_MARGIN << LEFT_UP_CORNER << horizontal_frame << RIGHT_UP_CORNER << NEW_LINE;
  auto add_line = [&]() {
      ret << LEFT_MARGIN << LEFT_JOINER << line_left_style;
      size_t space_size = width;
      for (size_t i = 0; i < space_size; i++) {
        ret << MIDDLE_HORIZONTAL;
      }
      ret << line_right_style;
      space_size = protocol_max - width;
      for (size_t i = 0; i < space_size; i++) {
        ret << MIDDLE_HORIZONTAL;
      }
      ret << RESET << RIGHT_JOINER << NEW_LINE;
    };
  auto add_meta =
    [&](bool _is_father,
      bool _list_end,
      bool _mode_end,
      const InterfacePrecursorTree & _father_tree,
      std::string target) {
      std::string index, trunk, branches, flowers;
      size_t space_size = protocol_max -
        (1 + width + target.size());  // 1:左边框
      if (_father_tree.size() > 1) {
        size_t trunk_size = _father_tree.size() - 1;
        for (size_t i = 0; i < trunk_size; i++) {
          trunk += (_father_tree.at(i) ? VERTICAL_FRAME : SPACE) + SPACE;
        }
        space_size -= (trunk_size * 2);
      }
      if (_is_father) {
        index = SPACE;
        if (!_father_tree.empty()) {
          branches =
            ((_list_end ||
            _mode_end) ? LEFT_DOWN_CORNER : MIDDLE_JOINER_RIGHT) + HORIZONTAL_FRAME +
            LIST_BIFURCATION;
          space_size -= 3;
        } else {
          branches = LIST_ROOT;
          space_size -= 1;
        }
        flowers = list_module_style + target;
      } else {
        index = std::to_string(++interface_index);
        branches = (_mode_end ? LEFT_DOWN_CORNER : MIDDLE_JOINER_RIGHT) + HORIZONTAL_FRAME +
          LIST_ARROW;
        space_size -= 3;
        if ((target == "state") ||
          (target == "data") ||
          (target == "params"))
        {
          flowers = list_variable_style + target;
        } else {
          flowers = list_function_style + target + ForeCOLOR_CYAN + LEFT_BRACKETS + RIGHT_BRACKETS;
          space_size -= 2;
        }
      }
      ret << LEFT_MARGIN << VERTICAL_FRAME << line_number_style << std::setw(width) << index <<
        list_tree_style << trunk << branches << flowers;
      for (size_t i = 0; i < space_size; i++) {
        ret << SPACE;
      }
      ret << RESET << VERTICAL_FRAME << NEW_LINE;
    };
  for (auto it_Ptr = module_list.rbegin(); it_Ptr != module_list.rend(); ++it_Ptr) {
    if (it_Ptr != module_list.rbegin() && it_Ptr->first && it_Ptr->second.first.empty()) {
      add_line();
    }
    bool list_end = true, mode_end = true;
    InterfacePrecursorTree father_tree(it_Ptr->second.first.size(), false);
    if (!father_tree.empty()) {
      for (auto next_ptr = it_Ptr + 1; next_ptr != module_list.rend(); next_ptr++) {
        if (next_ptr->first) {
          if (next_ptr->second.first.empty()) {
            break;
          }
          list_end = false;
        }
        if (mode_end &&
          (it_Ptr->second.first == next_ptr->second.first))
        {
          mode_end = false;
        }
        next_ptr->second.first.push_back(next_ptr->second.second);
        size_t now_size = (next_ptr->second.first.size() < it_Ptr->second.first.size()) ?
          next_ptr->second.first.size() : it_Ptr->second.first.size();
        for (size_t i = 1; i < now_size; i++) {
          size_t now_index = i - 1;
          if (!father_tree.at(now_index)) {
            if ((it_Ptr->second.first.at(now_index) == next_ptr->second.first.at(now_index)) &&
              (it_Ptr->second.first.at(i) != next_ptr->second.first.at(i)))
            {
              father_tree.at(now_index) = true;
            }
          }
        }
        next_ptr->second.first.pop_back();
      }
    }
    add_meta(it_Ptr->first, list_end, mode_end, father_tree, it_Ptr->second.second);
  }
  ret << LEFT_MARGIN << LEFT_DOWN_CORNER << horizontal_frame << RIGHT_DOWN_CORNER << NEW_LINE;
  return ret.str();
}

void Interface::HelpDocument(const std::string _function)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string function = '%s')",
    _function.c_str());
  DEBUG("%s", funs.c_str());
  toml::value root = interface_toml_;
  std::string frontlabel(""), label("");
  std::vector<std::string> interface;
  bool is_type = false;
  auto label_addc = [&](char c) {
      if (!is_type && (c >= 'A' && c <= 'Z')) {
        if (!label.empty()) {
          label += '_';
        }
        label += std::tolower(c);
      } else {
        label += c;
      }
    };
  auto godeep_toml = [&]() -> bool {
      if ((root.is_uninitialized()) || (!root.is_table()) || (root.count(label) == 0)) {
        return false;
      } else {
        toml::value _root = toml::find(root, label);
        root = _root;
        return true;
      }
    };
  for (auto c : _function) {
    if (c == '_') {
      if (godeep_toml()) {
        interface.push_back(label);
        if (!is_type && (label == interface_type)) {
          is_type = true;
        }
        label = "";
        continue;
      } else {
        ERROR(
          "%s \n%s\n\tGet list data(%s) failed.", this->logger_.c_str(),
          funs.c_str(), label.c_str());
        return;
      }
    }
    label_addc(c);
  }
  if (interface.empty()) {
    ERROR("%s \n%s\n\tParams failed(null).", this->logger_.c_str(), funs.c_str());
  }
  interface.erase(interface.begin());
  if (label == interface_list) {
    std::cout << this->GetList(interface, root) << std::endl;
  } else if (label == interface_tree) {
    std::cout << this->GetTree(interface, root) << std::endl;
  } else {
    frontlabel = interface.empty() ? "" : interface.back();
    interface.push_back(label);
    toml::value _root = toml::find(root, label);
    if (is_type ||
      (label == interface_state) ||
      (label == interface_data))
    {
      std::cout << this->GetDescribe(_root) << std::endl;
      std::cout << this->GetDemo(_root) << std::endl;
    } else if (label == interface_help) {
      std::cout << this->GetDescribe(_root) << std::endl;
    } else {
      std::cout << this->GetConstraint(interface, _root) << std::endl;
      std::cout << this->GetDescribe(_root) << std::endl;
      std::cout << this->GetDemo(_root) << std::endl;
    }
  }
}

}   // namespace cyberdog_visual_programming_terminal
