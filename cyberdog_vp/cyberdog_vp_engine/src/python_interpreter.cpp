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

#include "cyberdog_vp_engine/python_interpreter.hpp"

namespace cyberdog_visual_programming_engine
{
py::object get_module_header;                     /*!< 获取模块头部信息 */
py::object get_task_header;                       /*!< 获取任务头部信息 */
py::object decorate_body;                         /*!< 装饰body */
py::object generate_derivative_file;              /*!< 生成衍生文件 */
py::object IPythonDemo;                           /*!< 用于演示及单步执行或调试 */
bool PythonInterpreter::Init()
{
  py::module ament_index_python_packages = py::module::import("ament_index_python.packages");
  py::object get_package_share_directory = ament_index_python_packages.attr(
    "get_package_share_directory");
  py::module utils = py::module::import("mi.cyberdog_vp.utils");
  auto set_object = [&](py::object & _obj, std::string _fun) -> bool {
      if (!py::hasattr(utils, _fun.c_str())) {
        ERROR("'%s()' function not found in 'utils' module", _fun.c_str());
        return false;
      }
      _obj = utils.attr(_fun.c_str());
      return true;
    };
  if (!set_object(get_module_header, "get_module_header") ||
    !set_object(get_task_header, "get_task_header") ||
    !set_object(decorate_body, "decorate_body") ||
    !set_object(generate_derivative_file, "generate_derivative_file"))
  {
    return false;
  }
  py::module IPython_lib_demo = py::module::import("IPython.lib.demo");
  IPythonDemo = IPython_lib_demo.attr("Demo");
  return true;
}

bool PythonInterpreter::GetModuleHeader(
  const std::string & _interface,
  const std::string & _describe,
  const std::string & _body,
  std::vector<std::string> & _header,
  std::vector<std::string> & _import_module)
{
  try {
    INFO("%s()...", std::string(__FUNCTION__).c_str());
    auto try_import = [&](const std::string & _str) {
        std::vector<std::string> vct = GetVector(_str, ' ');  // from A import B
        if ((vct.size() > 1) &&
          (((vct.at(0) == "from") && (vct.at(2) == "import")) ||
          ((vct.at(0) == "import"))))
        {
          if (vct.at(1).find(PYTHON_PREFIX) == 0) {
            if (std::find(
                _import_module.begin(), _import_module.end(),
                vct.at(1)) == _import_module.end())
            {
              _import_module.push_back(vct.at(1));
            }
          }
        }
      };
    _import_module.clear();
    _header = LICENSES;
    py::object result_py = get_module_header(_interface, _describe, _body);
    std::string meta_str = "";
    for (auto meta : result_py) {
      meta_str = meta.cast<std::string>();
      _header.push_back(meta_str);
      try_import(meta_str);
    }
    return true;
  } catch (const std::exception & e) {
    ERROR("[%s()] error:%s", std::string(__FUNCTION__).c_str(), e.what());
  }
  return false;
}

bool PythonInterpreter::GetTaskHeader(
  const std::string & _id, const std::string & _body,
  std::vector<std::string> & _header,
  std::vector<std::string> & _import_module)
{
  try {
    INFO("%s()...", std::string(__FUNCTION__).c_str());
    auto try_import = [&](const std::string & _str) {
        std::vector<std::string> vct = GetVector(_str, ' ');  // from A import B
        if ((vct.size() > 1) &&
          (((vct.at(0) == "from") && (vct.at(2) == "import")) ||
          ((vct.at(0) == "import"))))
        {
          if (vct.at(1).find(PYTHON_PREFIX) == 0) {
            if (std::find(
                _import_module.begin(), _import_module.end(),
                vct.at(1)) == _import_module.end())
            {
              _import_module.push_back(vct.at(1));
            }
          }
        }
      };
    _import_module.clear();
    _header = LICENSES;
    py::object result_py = get_task_header("True", _id, _body);
    std::string meta_str = "";
    for (auto meta : result_py) {
      meta_str = meta.cast<std::string>();
      _header.push_back(meta_str);
      try_import(meta_str);
    }
    return true;
  } catch (const std::exception & e) {
    ERROR("[%s()] error:%s", std::string(__FUNCTION__).c_str(), e.what());
  }
  return false;
}

bool PythonInterpreter::DecorateBody(std::string & _body)
{
  try {
    INFO("%s()...", std::string(__FUNCTION__).c_str());
    py::object result_py = decorate_body(_body);
    std::ostringstream body_py;
    for (auto meta : result_py) {
      body_py << meta.cast<std::string>() << std::endl;
    }
    _body = body_py.str();
    return true;
  } catch (const std::exception & e) {
    ERROR("[%s()] error:%s", std::string(__FUNCTION__).c_str(), e.what());
  }
  return false;
}

bool PythonInterpreter::GenerateDerivativeFile(const std::string & _source_file)
{
  try {
    INFO("%s()...", std::string(__FUNCTION__).c_str());
    return generate_derivative_file(_source_file).cast<bool>();
  } catch (const std::exception & e) {
    ERROR("[%s()] error:%s", std::string(__FUNCTION__).c_str(), e.what());
  }
  return false;
}

}   // namespace cyberdog_visual_programming_engine
