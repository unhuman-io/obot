add_library(obot_test_main STATIC test_main.cpp)
target_include_directories(obot_test_main SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/third_party/doctest)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # doctest includes <ciso646>, which libstdc++ flags with a #warning in C++20.
  target_compile_options(obot_test_main PUBLIC -Wno-\#warnings)
endif()

function(obot_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE obot_rt obot_test_main obot_warnings)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

obot_test(test_can)
obot_test(test_ipc)
obot_test(test_drivers)
obot_test(test_usbrt)
obot_test(test_setpoint_filter)
obot_test(test_config)
obot_test(test_runtime)

# The shipped robot configuration must load and fit its buses.
add_test(NAME config_obot3_check
         COMMAND obot-rtd --check --config ${PROJECT_SOURCE_DIR}/../config/obot3.toml)

add_test(NAME config_obot1_check
         COMMAND obot-rtd --check --config ${PROJECT_SOURCE_DIR}/../config/obot1.toml)

# The whole daemon, two seconds on built-in simulated actuators.
add_test(NAME daemon_sim_smoke
         COMMAND obot-rtd --sim --no-rt --duration 2 --shm-name obot_ctest_smoke
                 --config ${PROJECT_SOURCE_DIR}/../config/obot3.toml)
add_test(NAME daemon_sim_smoke_obot1
         COMMAND obot-rtd --sim --no-rt --duration 2 --shm-name obot_ctest_smoke1
                 --config ${PROJECT_SOURCE_DIR}/../config/obot1.toml)
