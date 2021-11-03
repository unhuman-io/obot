#!/bin/bash

rosdistro=`ls /opt/ros`
if  [ $rosdistro == 'noetic' ]; then
    set -eo pipefail
    python3=3
fi
echo $rosdistro

cd $(dirname $0)/..
sudo apt install -y python$python3-rosdep libudev-dev libyaml-cpp-dev libeigen3-dev
. /opt/ros/$rosdistro/setup.bash
cd catkin-ws
if  [ $rosdistro == 'melodic' ]; then
    sudo rosdep init
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list  # just in case
    rosdep update
    sudo rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
    exit 0
else
    sudo rosdep install --from-paths src --ignore-src -r -y # --os=ubuntu:focal --rosdistro melodic
fi