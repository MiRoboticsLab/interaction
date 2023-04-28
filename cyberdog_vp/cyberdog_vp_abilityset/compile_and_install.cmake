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
# 功能说明: 编译和安装 sdk 库
#
function(compile_and_install_sdk)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Compiling and installing SDK ...")
  endif()
  file(GLOB_RECURSE _lib_file_list RELATIVE ${PROJECT_SOURCE_DIR} src/sdk/*.cpp)    # 以相对路径方式递归包含目标文件
  list(LENGTH _lib_file_list _lib_file_list_size)
  if(NOT _lib_file_list_size)
    message("\nLib flag is invalid(file list size ${_lib_file_list_size})")
    return()
  endif()
  set(_lib_name ${PROJECT_NAME})
  include_directories(include)
  add_library("${_lib_name}" SHARED
    ${_lib_file_list}
  )
  target_compile_features("${_lib_name}" PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories("${_lib_name}" PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies("${_lib_name}"
    rclcpp
    builtin_interfaces
    tf2
    tf2_ros
    sensor_msgs
    nav_msgs
    std_srvs
    ament_index_cpp
    protocol
    params
    cyberdog_common
    rapidjson
  )
  target_link_libraries("${_lib_name}"
    ${cyberdog_log_LIBRARIES}
  )
  install(DIRECTORY include/ DESTINATION include)
  install(
    TARGETS "${_lib_name}"
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
  ament_export_libraries(
    "${_lib_name}"
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )
  if(_ARG_LOG)
    message("${_lib_name} SDK compiled and installed.")
  endif()
endfunction()

#
# 功能说明: 获取 lib 名称（通过读取 api 文件，与其保持统一）
#
function(get_api_name lib_name_ is_ok_)
  set(api_file ${PROJECT_SOURCE_DIR}/src/api/main.cpp)
  set(${_lib_name} "" PARENT_SCOPE)
  set(${is_ok_} False PARENT_SCOPE)
  if(EXISTS "${api_file}")
    file(STRINGS ${api_file} data_list)
    list(LENGTH data_list data_list_size)
    if(data_list_size)
      foreach(data_var IN LISTS data_list)
        if(${data_var} MATCHES "^(.)?(PYBIND11_MODULE\\()+(.)+(\\))+(.)*$")
          string(REGEX REPLACE "^.*PYBIND11_MODULE\\((.*), m\\)" "\\1" _lib_name "${data_var}")
          break()
        endif()
      endforeach()
      if("${_lib_name}" STREQUAL "")
        message("API file is invalid(file line size ${data_list_size})。")
      else()
        set(${is_ok_} True PARENT_SCOPE)
        set(${lib_name_} "${_lib_name}" PARENT_SCOPE)
      endif()
    else()
      message("API file is null.")
    endif()
  else()
    message("API file is not exists.")
    message("API file required: ${api_file}.")
  endif()
endfunction()

#
# 功能说明: 修改目标依赖项私有()
# ament_target_dependencies 会将依赖项声明为私有，但这是 pybind11 所需的，所以复制粘贴
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
# 功能说明: 编译和安装 api 库
#
function(compile_and_install_api)
  cmake_parse_arguments(_ARG "LOG" "" "" ${ARGN})
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR "compile_and_install_node() Must be called before ament_package().")
  endif()
  if(_ARG_LOG)
    message("Compiling and installing API ...")
  endif()
  file(GLOB_RECURSE _lib_file_list RELATIVE ${PROJECT_SOURCE_DIR} src/sdk/*.cpp)    # 以相对路径方式递归包含目标文件
  list(LENGTH _lib_file_list _lib_file_list_size)
  if(NOT _lib_file_list_size)
    message("\nLib flag is invalid(file list size ${_lib_file_list_size})")
    return()
  endif()
  file(GLOB_RECURSE _api_file_list RELATIVE ${PROJECT_SOURCE_DIR} src/api/*pp)      # 以相对路径方式递归包含目标文件{*.hpp, *.cpp}
  list(LENGTH _api_file_list _api_file_list_size)
  if(NOT _api_file_list_size)
    message("\nLib flag is invalid(file list size ${_api_file_list_size})")
    return()
  endif()
  set(_api_name "")
  get_api_name(_api_name _is_ok)
  if(NOT ${_is_ok})
    message("API lib name is invalid.")
    return()
  endif()

  pybind11_add_module("${_api_name}"
    ${_lib_file_list}
    ${_api_file_list}
  )
  target_link_libraries("${_api_name}" PRIVATE
    ${CMAKE_INSTALL_PREFIX}/lib/libcyberdog_log.so
  )

  # add_library(${_api_name} SHARED
  #   ${_lib_file_list}
  #   ${_api_file_list}
  # )

  # target_link_libraries(${_api_name} PRIVATE
  #   pybind11::pybind11
  #   pybind11::module
  #   pybind11::lto
  #   pybind11::windows_extras
  #   ${cyberdog_log_LIBRARIES}
  #   )

  # pybind11_extension(${_api_name})
  # if(NOT MSVC AND NOT ${CMAKE_BUILD_TYPE} MATCHES Debug|RelWithDebInfo)
  #     # Strip unnecessary sections of the binary on Linux/macOS
  #     pybind11_strip(${_api_name})
  # endif()

  # set_target_properties(${_api_name} PROPERTIES CXX_VISIBILITY_PRESET "hidden"
  #                                          CUDA_VISIBILITY_PRESET "hidden")

  target_compile_features("${_api_name}" PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories("${_api_name}" PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies_private("${_api_name}"
    rclcpp
    ament_index_cpp
    protocol
    tf2
    tf2_ros
    params
    nav_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    cyberdog_common
    rapidjson
    pybind11::module
    cyberdog_log
  )
  install(
    TARGETS "${_api_name}"
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
  if("${PYTHON_INSTALL_DIR}" STREQUAL "")
  set(PYTHON_INSTALL_DIR "lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages")
  endif()
  install(
    TARGETS ${_api_name}
    DESTINATION ${PYTHON_INSTALL_DIR}/mi/cyberdog_vp
  )
  ament_export_libraries(
    "${_api_name}"
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )

  if(_ARG_LOG)
    message("${_api_name} API compiled and installed.")
  endif()
endfunction()
