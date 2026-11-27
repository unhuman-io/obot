# The obot Python tools (client, `obot` command, operator web page, recording) for robots
# whose own OS has no recent Python. It reaches the daemon through shared memory:
#
#   docker build -f docker/Dockerfile -t obot-tools .
#   docker run --rm -it --ipc=host --network=host \
#       -v /etc/obot:/etc/obot:ro --device /dev/video0 -v /dev/input:/dev/input:ro \
#       --group-add "$(getent group obot | cut -d: -f3)" \
#       obot-tools obot ui --joystick
FROM python:3.12-slim-bookworm

RUN pip install --no-cache-dir --upgrade pip
COPY python /src/python
RUN pip install --no-cache-dir "/src/python[ui,log]" && rm -rf /src

ENV OBOT_CONFIG=/etc/obot/robot.toml
EXPOSE 8080
CMD ["obot", "ui"]
