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

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include <cyberdog_common/cyberdog_log.hpp>

#include <iostream>
#include <vector>
#include <string>

int main(int _argc, char ** _argv)
{
  try {
    namespace py = pybind11;
    LOGGER_MAIN_INSTANCE("Terminal");
    INFO("Cyberdog visual programming terminal node started.");
    {
      py::scoped_interpreter guard{};
      {  // python 3.6
        std::vector<std::string> parameters;
        for (int i = 0; i < _argc; i++) {
          parameters.push_back(_argv[i]);
        }
        int argc = parameters.size();
        wchar_t ** argv = new wchar_t *[argc];
        for (int i = 0; i < argc; i++) {
          size_t size = parameters.at(i).size() + 1;
          argv[i] = new wchar_t[size];
          mbstowcs(argv[i], const_cast<char *>(parameters.at(i).c_str()), size);
        }
        PySys_SetArgv(argc, argv);
      }
      // {  // python 3.8
      //   py::module os = py::module::import("os");
      //   py::module sys = py::module::import("sys");
      //   py::object argv = sys.attr("argv");
      //   py::object argv_append = argv.attr("append");
      //   for (int i = 0; i < _argc; i++) {
      //     argv_append(_argv[i]);
      //   }
      // }
      py::object interaction = py::module::import("mi.cyberdog_vp.interaction");
    }
    INFO("Visual programming terminal node stopped.");
    return 0;
  } catch (const std::exception & e) {
    ERROR("Creating terminal failed, %s", e.what());
  }
  return -1;
}
