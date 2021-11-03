#!/bin/bash
set -eo pipefail

# ./add_freebot_rootfs.sh $(cat $HOME/jetson_nano/module_headers/kernel/kernel-4.9/include/config/kernel.release)

echo "********************** Adding freebot stuff ****************************"

if [[ -z $branch ]]; then
    branch=main
fi
echo "repo branch $branch"

cd rootfs
sudo cp /usr/bin/qemu-aarch64-static usr/bin
sudo cp --remove-destination -L /etc/resolv.conf etc/
sudo systemd-nspawn -D . -M tmpjetson --resolv-conf off --pipe /bin/bash << EOF
set -eo pipefail
apt update || true           # note some nvidia does not have a release file error
apt upgrade -y || true
apt install -y curl || true  # note some blueman error
cd /usr/src/linux-headers-${1}/kernel-4.9
make scripts -j10   # fix scripts that were not compiled correctly
add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted
echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt update || true
apt install -y ros-melodic-ros-base || true # same errors as above
curl https://raw.githubusercontent.com/unhuman-io/freebot/main/install-freebot.sh > install-freebot.sh
chmod +x install-freebot.sh
./install-freebot.sh    # note won't really configure dkms build correct until system restart
# freebot system build
cd /usr/local/src
sh -c 'curl https://storage.googleapis.com/git-repo-downloads/repo > repo'
chmod a+rx repo
git config --global user.name "Root"
git config --global user.email "root@root.com"
git config --global color.ui true
./repo init -b main -u https://github.com/unhuman-io/freebot-manifest
./repo sync
./freebot/install_freebot_build_deps.sh
./freebot/build_freebot.sh
EOF
sudo rm usr/bin/qemu-aarch64-static
sudo rm etc/resolv.conf