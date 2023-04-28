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
# 功能说明: 修改目标依赖项私有(ament_target_dependencies 会将依赖项声明为私有，但这是 pybind11 所需的，所以复制粘贴)
#
function(ament_target_dependencies_private target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    set(definitions "")
    set(include_dirs "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARGN})
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      list_append_unique(definitions ${${package_name}_DEFINITIONS})
      list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
      list(APPEND libraries ${${package_name}_LIBRARIES})
      list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
    endforeach()
    target_compile_definitions(${target}
      PUBLIC ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_include_directories(${target}
      PUBLIC ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target} PRIVATE
      ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()

#
# 功能说明: 编译和安装 节点
#
function(compile_and_install_node)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Compiling and installing ${PROJECT_NAME} node ...")
  endif()
  include_directories(include)
  file(GLOB_RECURSE _node_file_list RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)  # 以相对路径方式递归包含目标文件
  list(LENGTH _node_file_list _node_file_list_size)
  if(NOT _node_file_list_size)
    message("\nNode flag is invalid(file list size ${_node_file_list_size})")
    return()
  endif()
  set(target_node ${PROJECT_NAME})
  add_executable(${target_node} ${_node_file_list})

  target_link_libraries(${target_node} PRIVATE
    pybind11::embed
    ${CMAKE_INSTALL_PREFIX}/lib/libcyberdog_log.so
  )

  target_compile_features(${target_node} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${target_node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies_private(${target_node}
    ament_index_cpp
    rclcpp
    sensor_msgs
    protocol
    params
    cyberdog_common
    cyberdog_machine
    rapidjson
    pybind11::embed
  )
  install(TARGETS ${target_node}
    DESTINATION lib/${PROJECT_NAME})
  if(_ARG_LOG)
    message("Node ${_node_flag} compiled and installed.")
  endif()
endfunction()
