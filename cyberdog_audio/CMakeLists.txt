cmake_minimum_required(VERSION 3.8)
project(cyberdog_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(xpack REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_machine REQUIRED)
find_package(motion_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(user_info_manager REQUIRED)
find_package(cyberdog_debug REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  cyberdog_common
  xpack
  protocol
  cyberdog_machine
  motion_utils
  ament_index_cpp
  user_info_manager
  cyberdog_debug
)

# SET(RUN_LCMDATA_CMAKE_FILE "${CMAKE_BINARY_DIR}/lcm_data.cmake")
# FILE(WRITE ${RUN_LCMDATA_CMAKE_FILE}
# "SET(ENV{LANG en})\n")
# FILE(APPEND ${RUN_LCMDATA_CMAKE_FILE}
# "EXECUTE_PROCESS(COMMAND lcm-gen -x ${CMAKE_SOURCE_DIR}/lcm_data.lcm WORKING_DIRECTORY \"${CMAKE_BINARY_DIR}\")\n")

execute_process(COMMAND lcm-gen -x ${CMAKE_SOURCE_DIR}/lcm_data.lcm WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")

include_directories(include ${CMAKE_BINARY_DIR})
add_executable(cyberdog_audio src/main.cpp
  src/cyberdog_audio.cpp)
target_link_libraries(cyberdog_audio
${cyberdog_log_LIBRARIES}
lcm
)
target_include_directories(cyberdog_audio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cyberdog_audio PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cyberdog_audio
  ${dependencies}
)

install(TARGETS cyberdog_audio
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
