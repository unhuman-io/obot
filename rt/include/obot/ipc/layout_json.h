// Machine-readable description of the shared memory layout, used to check that other
// languages' views of it (python/src/obot/shm/layout.py) agree with this build.
#pragma once

#include <string>

namespace obot::ipc {

std::string layout_json();

}  // namespace obot::ipc
