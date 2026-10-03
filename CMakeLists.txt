cmake_minimum_required(VERSION 3.22)
project(MC02Infantry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# control/：纯算法层，固件和主机单测共用同一份源码
add_library(control STATIC
  control/src/five_bar_leg.cpp
)
target_include_directories(control PUBLIC control/include)
target_compile_options(control PRIVATE -Wall -Wextra -Wpedantic -Wdouble-promotion -Wshadow)

# drivers/：外设协议编解码（CAN 帧打包/解包），同样无硬件依赖
add_library(drivers STATIC
  drivers/src/mit_motor.cpp
)
target_include_directories(drivers PUBLIC drivers/include)
target_compile_options(drivers PRIVATE -Wall -Wextra -Wpedantic -Wdouble-promotion -Wshadow -Wconversion)

# 主机单测（固件交叉编译时关闭）
option(MC02_BUILD_TESTS "Build host unit tests" ON)
if(MC02_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(control_tests tests/test_five_bar_leg.cpp tests/test_mit_motor.cpp)
  target_include_directories(control_tests PRIVATE tests)
  target_link_libraries(control_tests PRIVATE control drivers GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(control_tests)
endif()
