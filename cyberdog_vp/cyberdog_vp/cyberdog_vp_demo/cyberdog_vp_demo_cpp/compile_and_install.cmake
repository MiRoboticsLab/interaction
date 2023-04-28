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
# 功能说明: 编译和安装 节点
#
function(compile_and_install_node target)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Compiling and installing ${PROJECT_NAME} node ...")
  endif()

  include_directories(include)
  add_executable(${target}
    src/${target}.cpp)
  target_link_libraries(${target}
    ${CMAKE_INSTALL_PREFIX}/lib/libcyberdog_log.so
    ${CMAKE_INSTALL_PREFIX}/lib/libcyberdog_vp_abilityset.so
  )
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target}
    ament_index_cpp
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    tf2_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    protocol
    cyberdog_common
    cyberdog_vp_abilityset
  )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})

  if(_ARG_LOG)
    message("Node ${_node_flag} compiled and installed.")
  endif()
endfunction()
