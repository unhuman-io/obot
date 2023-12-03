#!/bin/bash
set -eo pipefail

# build assuming directory setup from repo manifist

cd $(dirname $0)/..

cores=$(nproc --all)

pushd motor-realtime
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_PYTHON_API=ON -DCMAKE_INSTALL_PREFIX=artifacts -DINSTALL_COMPLETION=OFF ..
make -j $cores
make install
popd

pushd obot-controller/obot_g474
make CONFIG=motor_enc -j $cores
popd


# pushd 3rdparty/rbdl
# mkdir -p build && cd build
# cmake -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USE_ROS_URDF_LIBRARY=OFF -DCMAKE_INSTALL_PREFIX=artifacts ..
# make -j9
# make install
# popd

# pushd obot-realtime
# mkdir -p build && cd build
# cmake -DRBDL_PATH=../../3rdparty/rbdl/build/artifacts -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
# make -j9
# popd

# . /opt/ros/*/setup.bash
# pushd catkin-ws
# catkin_make -DOBOT_INCLUDE_DIR=../../obot-realtime/include
# popd
