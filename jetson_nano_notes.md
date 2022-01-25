# Jetson Nano notes

Currently I'm using the jetson nano developer kit 4gb "b" version. This also 
seems to be referred to as 300 or b00 or b01 or 945-13450-0000-100. I have a number of recommended parts.

| Item                              | Part number               | Quantity  |
| --------------------------------- | ------------------------- | --------- |
| Jetson nano kit b01 4gb           | 945-13450-0000-100        | 1         |
| Kingston canvas go sd card        | SDCG2/64GB                | 1         |
| Intel 8265 M.2 wifi card          | 8265.NGWMG.NV             | 1         |
| Molex u.fl antennas               | 538-204281-1100           | 2         |
| 5V 5.5 x 2.1mm 10W power supply   |                           | 1         |

I maintain an rt preempt image for this board. It also includes obot code and dependencies prebuilt. The current version is built for Nvidia L4T release R32.6.1: [download R32.6.1 4gb b0x](https://drive.google.com/file/d/16DrsnGvGBAuIRJje9GAWx70w419WVTM1).

To load the image I have been using [balenaEtcher](https://www.balena.io/etcher/), per the Nvidia recommendations. The image is provided in a state that needs to be configured for user name and password, hostname, location, and wifi connection. It can be configured either by connecting a keyboard and monitor or via a micro usb connection from a remote PC. The keyboard and monitor configuration is straightforward. When using the micro usb approach you connect via a serial terminal connection in Linux the command is `sudo screen /dev/ttyACM0`. In Windows I would use putty and try to connect to the appropriate `COM` port. Some notes on the configuration steps:
- During the setup I configured with dummy network since the wifi doesn't seem to work at first. It fails configuration then select do not configure at this time. Then after reboot I do `sudo nmtui` to configure a wifi connection. Once wifi is configured you will now be able to communicate via ssh and the jetson nano hostname, `e.g. ssh r1.local`.
- I switch to text only with `sudo systemctl set-default multi-user.target`
- For the ac8265 I found that `sudo iwconfig wlan0 power off` is useful.