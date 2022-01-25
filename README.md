# OBOT

The open source robot project. See the wiki for a project description: https://github.com/unhuman-io/obot/wiki

# Host PC software

Obot is designed to run on linux both on small computers like the Nvidia 
Jetson Nano and the Raspberry pi, or larger computers like laptops. A debian 
based linux distrubution is assumed, but Obot can be run on other linux 
distributions as well. Some subcomponents are necessary in order to run 
hardware. They can be built from source or installed from binaries using the 
method described below.

Install linux kernel headers appropriate for your installation. For example:
```shell
sudo apt install -y linux-headers-$(uname -r)
```
or:
```shell
sudo apt install -y raspberrypi-kernel-headers
```
then:
```shell
curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-obot.sh > install-obot.sh
chmod +x install-obot.sh
./install-obot.sh
```

then to later update
```shell
update-obot
```

# Source

Software requirements include installing ros.

```shell
curl https://storage.googleapis.com/git-repo-downloads/repo > repo
chmod +x repo
./repo init -b main -u https://github.com/unhuman-io/obot-manifest # or git@github.com:unhuman-io/obot-manifest
./repo sync
```
A build script builds the project
```shell
./obot/install_obot_build_deps.sh
./obot/build_obot.sh
```

# Jetson Nano image

This is a prebuilt image with the preempt rt patch enabled. You can generate an image with the 
`make_jetson_nano.sh` script. And copy it to an SD card with balena etcher. Some additional notes 
after startup of the jetson nano are:
- I've used the usb serial interface to connect with the device rather than graphical, but either works
  - sudo screen /dev/ttyACM0
- During the setup I configured with dummy network since the wifi doesn't seem to work at first both for the intel ac8265 and the usb wifi included with the 2gb version. It fails configuration then select do not configure at this time. Then after reboot I do sudo nmtui to configure a wifi connection.
- I switch to text only with sudo systemctl set-default multi-user.target
- For the ac8265 I found that `sudo iwconfig wlan0 power off` is useful.

The image also includes obot software. Obot source is installed in `/usr/local/src`. You 
can change permissions with `sudo chown -R $USER:$USER /usr/local/src`. 

# Run

The first demo uses a Host PC with monitor and joystick to remote control move
the robot base. Video is transferred from the obot to the Host PC and data is
transferred via wifi. To run the demo, on the host PC:
```shell
./obot/jetbot_camera_display.sh
source catkin-ws/devel/setup.bash
roslaunch obot-ros obot_base.launch
```

on the Obot:
```shell
./obot/jetbot_camera_run.sh
source catkin-ws/devel/setup.bash
roslaunch obot-ros obot_hardware.launch
```
