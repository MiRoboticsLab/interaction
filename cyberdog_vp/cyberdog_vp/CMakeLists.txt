cmake_minimum_required(VERSION 3.8)
project(cyberdog_vp)

find_package(ament_cmake REQUIRED)

include(compile_and_install.cmake)
install_workspace()
install_script()

install(DIRECTORY workspace script config gadget pdf DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
