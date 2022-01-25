FROM ros:noetic-robot
RUN apt update && \
    apt install -y curl libudev-dev python git && \
    git config --global user.email "root@root.com" && \
    git config --global user.name "root" && \
    git config --global color.ui true && \
    curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-obot.sh > install-obot.sh && \
    chmod +x install-obot.sh && \
    ./install-obot.sh --no-driver && \
    curl https://storage.googleapis.com/git-repo-downloads/repo > repo && \
    chmod +x repo
