cmake_minimum_required(VERSION 3.22)
project(obot-rt VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(OBOT_RT_BUILD_TESTS "Build the tests" ON)
option(OBOT_RT_WERROR "Treat warnings as errors" OFF)
option(OBOT_RT_STATIC "Link the programs statically (runs on any aarch64 / x86_64 Linux, whatever its libc)" OFF)
set(OBOT_RT_SANITIZE "" CACHE STRING "Sanitizers to build with, e.g. address,undefined or thread")

include(GNUInstallDirs)
find_package(Threads REQUIRED)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/motor_messages/motor_messages.h")
  message(FATAL_ERROR "third_party/motor_messages is empty: run 'git submodule update --init'")
endif()

add_library(obot_warnings INTERFACE)
target_compile_options(obot_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
if(OBOT_RT_WERROR)
  target_compile_options(obot_warnings INTERFACE -Werror)
endif()
if(OBOT_RT_SANITIZE)
  target_compile_options(obot_warnings INTERFACE -fsanitize=${OBOT_RT_SANITIZE} -fno-omit-frame-pointer)
  target_link_options(obot_warnings INTERFACE -fsanitize=${OBOT_RT_SANITIZE})
endif()

add_library(obot_rt STATIC
  src/actuator/cubemars_driver.cpp
  src/actuator/obot_driver.cpp
  src/can/fake_bus.cpp
  src/can/socketcan_bus.cpp
  src/can/usbrt_bus.cpp
  src/ipc/layout_json.cpp
  src/ipc/shm_region.cpp
  src/robot/config.cpp
  src/robot/runtime.cpp
  src/robot/sim_factory.cpp
  src/rt/realtime.cpp
  src/sim/devices.cpp
)
target_include_directories(obot_rt PUBLIC include)
# The wire structs come from the firmware's own header; toml++ is only used by config.cpp.
target_include_directories(obot_rt SYSTEM PUBLIC third_party/motor_messages PRIVATE third_party/tomlplusplus)
target_link_libraries(obot_rt PUBLIC Threads::Threads PRIVATE obot_warnings)

foreach(app obot_rtd obot_sim_node obot_rt_latency)
  string(REPLACE "_" "-" exe ${app})
  add_executable(${exe} apps/${app}.cpp)
  target_link_libraries(${exe} PRIVATE obot_rt obot_warnings)
  if(OBOT_RT_STATIC)
    target_link_options(${exe} PRIVATE -static)
  endif()
  install(TARGETS ${exe} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

if(OBOT_RT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
