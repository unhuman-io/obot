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
This is a prebuilt image with the preempt rt patch enabled. It also includes freebot software.