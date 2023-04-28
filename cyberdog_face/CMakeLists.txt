cmake_minimum_required(VERSION 3.8)
project(cyberdog_face)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protocol REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(params REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(user_info_manager REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/cyberdog_face_entry.cpp)

target_link_libraries(${PROJECT_NAME}
  ${cyberdog_log_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  protocol
  ament_index_cpp
  params
  cyberdog_system
  cyberdog_common
  user_info_manager
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
