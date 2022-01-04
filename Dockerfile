FROM ros:noetic-robot
RUN apt update && \
    apt install -y curl libudev-dev python git && \
    git config --global user.email "root@root.com" && \
    git config --global user.name "root" && \
    git config --global color.ui true && \
    curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-freebot.sh > install-freebot.sh && \
    chmod +x install-freebot.sh && \
    ./install-freebot.sh --no-driver && \
    curl https://storage.googleapis.com/git-repo-downloads/repo > repo && \
    chmod +x repo
