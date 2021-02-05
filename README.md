# Freebot

See the wiki for a project description: https://github.com/unhuman-io/freebot/wiki

# Host PC software

```shell
curl https://raw.githubusercontent.com/unhuman-io/freebot/main/install-freebot.sh > install-freebot.sh
chmod +x install-freebot.sh
./install-freebot.sh
```

then to later update
```shell
update-freebot
```

# Source

```shell
sudo apt install repo
repo init -b main -u https://github.com/unhuman-io/freebot
repo sync
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

The image also includes freebot software. Freebot source is installed in `/usr/local/src`. You 
can change permissions with `sudo chown -R $USER:$USER /usr/local/src`. 