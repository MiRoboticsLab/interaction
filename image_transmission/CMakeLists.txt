cmake_minimum_required(VERSION 3.8)
add_definitions(-w)
project(image_transmission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(protocol)
find_package(jsoncpp REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(X11 REQUIRED)

include_directories(
  include
  /usr/local/include/webrtc_headers
  /usr/local/include/webrtc_headers/third_party/abseil-cpp
  /usr/include
  /usr/include/jsoncpp
)

link_directories(/usr/local/lib)

add_definitions(-D WEBRTC_POSIX)
add_definitions(-D WEBRTC_LINUX)

add_library(live_stream_broadcaster SHARED
  src/webrtc_conductor/json.cc
  src/webrtc_conductor/defaults.cc
  src/webrtc_conductor/video_adapter.cc
  src/webrtc_conductor/conductor.cpp
  src/live_stream_broadcaster.cpp
)
target_include_directories(live_stream_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(live_stream_broadcaster PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(live_stream_broadcaster
  rclcpp
  std_msgs
  protocol
  cyberdog_common
)
target_link_libraries(live_stream_broadcaster
  webrtc
  Threads::Threads
  ${X11_LIBRARIES}
  jsoncpp
)
ament_export_targets(live_stream_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs protocol cyberdog_common)

if(LOCAL_BUILD_TESTING)
  add_executable(test_webrtc_video_sender
    test/i420_creator.cpp
    test/test_main.cpp
  )
  target_include_directories(test_webrtc_video_sender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(test_webrtc_video_sender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(test_webrtc_video_sender
    rclcpp
  )
  target_link_libraries(test_webrtc_video_sender
    live_stream_broadcaster
  )
endif()

install(
  DIRECTORY include/image_transmission/
  DESTINATION include/image_transmission/
)

install(TARGETS live_stream_broadcaster
  EXPORT live_stream_broadcaster
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

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
