#!/bin/sh
# Builds obot-segd on the robot itself, against TensorRT and CUDA as NVIDIA installs them on
# a Jetson:  rt/gpu/build.sh [OUT]   (default ./obot-segd). Exit status 3: no TensorRT here.
set -eu
here="$(cd "$(dirname "$0")" && pwd)"
out="${1:-obot-segd}"
[ -e /usr/include/aarch64-linux-gnu/NvInfer.h ] || [ -e /usr/include/x86_64-linux-gnu/NvInfer.h ] || [ -e /usr/include/NvInfer.h ] || {
    echo "no TensorRT on this computer (on a Jetson: apt install libnvinfer-dev libnvonnxparsers-dev cuda-cudart-dev-10-2)" >&2
    exit 3
}
library="$(find /usr/local/cuda* -name libcudart.so 2>/dev/null | head -1)"
[ -n "$library" ] || { echo "no CUDA runtime (libcudart.so) under /usr/local/cuda*" >&2; exit 3; }
lib="$(dirname "$library")"
g++ -std=c++14 -O2 -Wall -Wno-deprecated-declarations "$here/obot_segd.cpp" -o "$out" \
    -L"$lib" -Wl,-rpath,"$lib" -lnvinfer -lnvonnxparser -lcudart
