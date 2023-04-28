cmake_minimum_required(VERSION 3.8)
project(cyberdog_vp_terminal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(python_cmake_module REQUIRED)

# find_package (Python3 COMPONENTS Interpreter Development)
# find_package(PythonInterp)
# if (PYTHONINTERP_FOUND)
#   if (PYTHON_VERSION_MAJOR EQUAL 3)
#     find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
#     find_package(PythonInterp 3)
#     find_package(PythonLibs 3 REQUIRED)
#     set(boost_python "boost_python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
#   else()
#     find_package(Boost COMPONENTS python)
#     find_package(PythonInterp)
#     find_package(PythonLibs REQUIRED)
#     set(boost_python "boost_python")
#   endif()
# else()
#   message("Python not found")
# endif()
# message(STATUS "Boost_python = ${boost_python}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(protocol REQUIRED)
find_package(params REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(rapidjson REQUIRED)

find_package(cyberdog_vp_abilityset REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)
find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

include(compile_and_install.cmake)
compile_and_install_node()
compile_and_install_api()

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

ament_export_include_directories(include)
ament_package()
