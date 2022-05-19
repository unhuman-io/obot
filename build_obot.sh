#!/bin/bash
set -eo pipefail

# build assuming directory setup from repo manifist

cd $(dirname $0)/..
pushd 3rdparty/rbdl
mkdir -p build && cd build
cmake -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USE_ROS_URDF_LIBRARY=OFF -DCMAKE_INSTALL_PREFIX=artifacts ..
make -j9
make install
popd

pushd obot-realtime
mkdir -p build && cd build
cmake -DRBDL_PATH=../../3rdparty/rbdl/build/artifacts -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j9
popd

. /opt/ros/*/setup.bash
pushd catkin-ws
catkin_make -DOBOT_INCLUDE_DIR=../../obot-realtime/include
popd