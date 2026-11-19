// The obot-controller wire structs, straight from the firmware's motor_messages
// repository (third_party/motor_messages). That header relies on its includer for the
// fixed-width integer types, so always include it through this one.
#pragma once

#include <stdint.h>  // IWYU pragma: keep

#include <motor_messages.h>  // IWYU pragma: export
