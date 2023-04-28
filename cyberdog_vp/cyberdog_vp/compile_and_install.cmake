# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# 功能说明: 获取工作空间（通过配置 环境文件）
#
function(get_workspace value_)
  set(params_toml "${PROJECT_SOURCE_DIR}/config/abilityset.toml")
  execute_process(
    COMMAND cat ${params_toml}
    COMMAND grep "workspace="
  OUTPUT_VARIABLE _target_line)
  string(REGEX REPLACE "\n$" "" _target_line "${_target_line}")
  string(REGEX REPLACE "workspace=\"(.*)\".*" "\\1" _workspace "${_target_line}")
  if(${_workspace} MATCHES "^\/.*$")
    if(NOT IS_DIRECTORY ${_workspace})
      file(MAKE_DIRECTORY ${_workspace})
    endif()
    set(${value_} ${_workspace} PARENT_SCOPE)
  else()
    set(${value_} ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME} PARENT_SCOPE)
  endif()
endfunction()


#
# 功能说明: 安装 工作空间
#
function(install_workspace)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Installing ${PROJECT_NAME} workspace ...")
  endif()
  set(workspace)
  get_workspace(workspace)
  install(DIRECTORY workspace DESTINATION ${workspace})
  if(_ARG_LOG)
    message("Node ${_node_flag} compiled and installed.")
  endif()
endfunction()


#
# 功能说明: 安装 脚本
#
function(install_script)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Installing ${PROJECT_NAME} script ...")
  endif()
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)
  find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
  if("${PYTHON_INSTALL_DIR}" STREQUAL "")
  set(PYTHON_INSTALL_DIR "lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages")
  endif()
  install(DIRECTORY script/
  DESTINATION ${PYTHON_INSTALL_DIR}/mi/${PROJECT_NAME})
  if(_ARG_LOG)
    message("Node ${_node_flag} compiled and installed.")
  endif()
endfunction()
