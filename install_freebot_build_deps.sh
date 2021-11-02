#!/bin/bash

rosdistro=`ls /opt/ros`
if  [ $rosdistro == 'noetic' ]; then
    set -eo pipefail
    python3=3
fi

cd $(dirname $0)/..
sudo apt install -y python$python3-rosdep libudev-dev libyaml-cpp-dev
. /opt/ros/$rosdistro/setup.bash
cd catkin-ws
if  [ $rosdistro == 'melodic' ]; then
    sudo rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
else
    sudo rosdep install --from-paths src --ignore-src -r -y # --os=ubuntu:focal --rosdistro melodic
fi