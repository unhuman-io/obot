#!/bin/bash

set -eo pipefail

# if command -v mkvirtualenv; then
#     echo "installing virtualenvwrapper"
#     sudo apt-get install virtualenvwrapper
#     echo "adding virtualenvwrapper.sh to .bashrc"
#     echo ". /usr/share/virtualenvwrapper/virtualenvwrapper.sh" >> ~/.bashrc
#     . ~/.bashrc
# fi


# . /usr/share/virtualenvwrapper/virtualenvwrapper.sh
# echo "yay"
# #mkvirtualenv obot

# echo "yay"

# source_line="workon obot"
# if grep "$source_line" ~/.bashrc; then
#     read -r -p "Would you like add 'workon obot' to your ~/.bashrc [y/N] " response
#     if [[ "$response" =~ ^([yY])$ ]]
#     then
#         echo $source_line >> ~/.bashrc
#     fi
# fi



# rosdistro=`ls /opt/ros`
# if  [ $rosdistro == 'noetic' ]; then
#     set -eo pipefail
#     python3=3
# fi
# echo $rosdistro

# cd $(dirname $0)/..
# sudo apt install -y python$python3-rosdep libudev-dev libyaml-cpp-dev libeigen3-dev
# . /opt/ros/$rosdistro/setup.bash
# cd catkin-ws
# if  [ $rosdistro == 'melodic' ]; then
#     sudo rosdep init
#     rosdep update
#     sudo rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
#     exit 0
# else
#     sudo rosdep init || true
#     rosdep update
#     rosdep install --from-paths src --ignore-src -r -y # --os=ubuntu:focal --rosdistro melodic
# fi