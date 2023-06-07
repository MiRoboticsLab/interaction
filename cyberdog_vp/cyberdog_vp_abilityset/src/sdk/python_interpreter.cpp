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

#include "cyberdog_vp_abilityset/python_interpreter.hpp"

namespace cyberdog_visual_programming_abilityset
{
py::object choreographer_dance_args;              // 编舞
py::object choreographer_dance_kwargs;            // 编舞
bool PythonInterpreter::Init()
{
  py::module ament_index_python_packages = py::module::import("ament_index_python.packages");
  py::object get_package_share_directory = ament_index_python_packages.attr(
    "get_package_share_directory");
  py::module choreographer = py::module::import("mi.cyberdog_vp.choreographer");
  auto set_object = [&](py::object & _obj, std::string _fun) -> bool {
      if (!py::hasattr(choreographer, _fun.c_str())) {
        ERROR("'%s()' function not found in 'choreographer' module", _fun.c_str());
        return false;
      }
      _obj = choreographer.attr(_fun.c_str());
      return true;
    };
  if ((!set_object(choreographer_dance_args, "dance_args")) ||
    (!set_object(choreographer_dance_kwargs, "dance_kwargs")))
  {
    return false;
  }
  return true;
}

MotionSequenceServiceResponse PythonInterpreter::Choreographer(
  const uint64_t _motion_id,
  const std::string _type,
  const py::args _args)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s, %s) ...",
    _type.c_str(),
    pyArgsToString(_args).c_str());
  INFO("%s", funs.c_str());
  MotionSequenceServiceResponse ret;
  ret.state.code = StateCode::fail;
  ret.state.describe = funs;
  try {
    ret = choreographer_dance_args(_motion_id, _type, _args).cast<MotionSequenceServiceResponse>();
  } catch (const std::exception & e) {
    ERROR("%s error:%s", funs.c_str(), e.what());
    ret.state.describe += "\n - " + std::string(e.what());
  }
  return ret;
}

MotionSequenceServiceResponse PythonInterpreter::Choreographer(
  const uint64_t _motion_id,
  const py::kwargs _kwargs)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(%s) ...",
    pyKwargsToString(_kwargs).c_str());
  INFO("%s", funs.c_str());
  MotionSequenceServiceResponse ret;
  ret.state.code = StateCode::fail;
  ret.state.describe = funs;
  try {
    ret = choreographer_dance_kwargs(_motion_id, _kwargs).cast<MotionSequenceServiceResponse>();
  } catch (const std::exception & e) {
    ERROR("%s error:%s", funs.c_str(), e.what());
    ret.state.describe += "\n - " + std::string(e.what());
  }
  return ret;
}
}   // namespace cyberdog_visual_programming_abilityset
