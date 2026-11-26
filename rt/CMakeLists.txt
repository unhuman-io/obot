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
# Release 1.0.3, by commit so that what is fetched cannot change.
set(OBOT_LIBUDEV_ZERO_REVISION "ee32ac5f6494047b9ece26e7a5920650cdf46655" CACHE STRING
    "libudev-zero revision fetched for static builds")
set(OBOT_RT_SANITIZE "" CACHE STRING "Sanitizers to build with, e.g. address,undefined or thread")

include(GNUInstallDirs)
find_package(Threads REQUIRED)

# --- motor-realtime: all communication with the actuators goes through this library -------
# Built from the submodule with the source list of the library's own minimal ("embedded")
# build: the motor manager and the transports, without motor_util and the rest.
set(MR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/motor-realtime")
if(NOT EXISTS "${MR}/include/motor_messages/motor_messages.h" OR NOT EXISTS "${MR}/3rdparty/obot-protocol/protocol_parser.cpp")
  message(FATAL_ERROR "third_party/motor-realtime is incomplete: run 'git submodule update --init --recursive'")
endif()
add_library(motor_realtime STATIC
  ${MR}/src/motor/motor_manager.cpp
  ${MR}/src/motor/motor.cpp
  ${MR}/src/motor/motor_can.cpp
  ${MR}/src/motor/motor_ip.cpp
  ${MR}/src/motor/motor_eth_l2.cpp
  ${MR}/src/motor/motor_uart.cpp
  ${MR}/src/motor/motor_uart_obot.cpp
  ${MR}/src/motor/motor_thread.cpp
  ${MR}/src/motor/realtime_thread.cpp
  ${MR}/src/motor/terminal.cpp
  ${MR}/3rdparty/obot-protocol/protocol_parser.cpp
)
target_include_directories(motor_realtime SYSTEM PUBLIC ${MR}/include ${MR}/3rdparty/obot-protocol)
target_compile_options(motor_realtime PRIVATE -w)   # not our code: its warnings are not ours to fix here
target_link_libraries(motor_realtime PUBLIC Threads::Threads)
# The realtime loop is the library's MotorThread; its fixed-size records have to hold as
# many motors as a robot can have joints (ipc::kMaxJoints).
target_compile_definitions(motor_realtime PUBLIC MAX_MOTORS=16)
# The library finds USB boards with libudev.
if(OBOT_RT_STATIC)
  # There is no static libudev to link (systemd does not provide one). libudev-zero is a
  # small implementation of the same interface that reads /sys directly.
  include(FetchContent)
  FetchContent_Declare(libudev_zero
    GIT_REPOSITORY https://github.com/illiliti/libudev-zero.git
    GIT_TAG ${OBOT_LIBUDEV_ZERO_REVISION})
  FetchContent_MakeAvailable(libudev_zero)   # sources only: it has no CMake build of its own
  enable_language(C)
  file(GLOB UDEV_ZERO_SOURCES ${libudev_zero_SOURCE_DIR}/udev*.c)
  add_library(udev_zero STATIC ${UDEV_ZERO_SOURCES})
  target_compile_options(udev_zero PRIVATE -w -D_XOPEN_SOURCE=700)
  # The library's header is udev.h; programs include <libudev.h>.
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/udev_zero)
  file(COPY_FILE ${libudev_zero_SOURCE_DIR}/udev.h ${CMAKE_CURRENT_BINARY_DIR}/udev_zero/libudev.h ONLY_IF_DIFFERENT)
  target_include_directories(udev_zero SYSTEM PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/udev_zero)
  target_link_libraries(motor_realtime PUBLIC udev_zero)
else()
  find_library(UDEV_LIBRARY udev)
  find_path(UDEV_INCLUDE_DIR libudev.h)
  if(NOT UDEV_LIBRARY OR NOT UDEV_INCLUDE_DIR)
    message(FATAL_ERROR "libudev is needed by motor-realtime: install libudev-dev (Debian, Ubuntu) or systemd-devel")
  endif()
  target_include_directories(motor_realtime SYSTEM PUBLIC ${UDEV_INCLUDE_DIR})
  target_link_libraries(motor_realtime PUBLIC ${UDEV_LIBRARY})
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
  src/actuator/obot_driver.cpp
  src/ipc/layout_json.cpp
  src/ipc/shm_region.cpp
  src/robot/config.cpp
  src/robot/runtime.cpp
  src/robot/motor_factory.cpp
  src/rt/realtime.cpp
  src/sim/devices.cpp
)
target_include_directories(obot_rt PUBLIC include)
# toml++ is only used by config.cpp.
target_include_directories(obot_rt SYSTEM PRIVATE third_party/tomlplusplus)
target_link_libraries(obot_rt PUBLIC motor_realtime Threads::Threads PRIVATE obot_warnings)

# The map of the surroundings and what is worked out from it: no motors, no threads.
add_library(obot_nav STATIC
  src/nav/voxel_map.cpp
  src/nav/floor.cpp
)
target_include_directories(obot_nav PUBLIC include)
target_link_libraries(obot_nav PRIVATE obot_warnings)

add_executable(obot-navd apps/obot_navd.cpp)
target_link_libraries(obot-navd PRIVATE obot_nav obot_warnings)
if(OBOT_RT_STATIC)
  target_link_options(obot-navd PRIVATE -static)
endif()
install(TARGETS obot-navd RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

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
