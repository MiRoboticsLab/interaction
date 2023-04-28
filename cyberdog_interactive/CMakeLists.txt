cmake_minimum_required(VERSION 3.8)
project(cyberdog_interactive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(protocol REQUIRED)
find_package(params REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_system REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/interactive.cpp
  src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  ${CMAKE_INSTALL_PREFIX}/lib/libcyberdog_log.so
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
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
  params
  cyberdog_common
  cyberdog_system
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
