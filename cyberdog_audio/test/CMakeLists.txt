cmake_minimum_required(VERSION 3.8)
project(audio_simulator)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
)

set(dependencies
  cyberdog_common
  xpack
)

add_executable(audio_simulator audio_simulator.cpp)
target_link_libraries(audio_simulator
lcm
${cyberdog_log_LIBRARIES}
)
ament_target_dependencies(audio_simulator ${dependencies})