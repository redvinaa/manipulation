# syntax = docker/dockerfile:1.3

ARG FROM_IMAGE=ros:humble

# Multi-stage for caching
FROM $FROM_IMAGE AS cacher

# Clone source
WORKDIR /home/ubuntu/manipulation
COPY . .

# Copy manifests for caching
RUN find . -mindepth 1 -maxdepth 2 -name "src" -type d -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/manipulation/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/manipulation && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/manipulation || true


# Multi-stage for building
FROM $FROM_IMAGE AS installer

# Config dependencies install
ARG DEBIAN_FRONTEND=noninteractive

# REMOVEME: packages need to be upgraded from kisak-mesa repo
# libegl-mesa0 libgbm1 libgl1-mesa-dev
# libgl1-mesa-dri libglapi-mesa libglx-mesa0 libllvm15
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt update && apt install -y \
        software-properties-common && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    apt update && apt upgrade -y \
        libegl-mesa0 \
        libgbm1 \
        libgl1-mesa-dev \
        libgl1-mesa-dri \
        libglapi-mesa \
        libglx-mesa0 \
        libllvm15

# Set root password
RUN echo "root:enjoy"|chpasswd

# Create new non-priviliged user
RUN useradd ubuntu --create-home --shell /bin/bash && \
    echo "ubuntu:enjoy"|chpasswd && \
    usermod -aG sudo,video,dialout ubuntu
USER root

## Install Eigen library
RUN git clone https://gitlab.com/libeigen/eigen.git /usr/include/eigen

# Install system dependencies with caching apt and pip package lists
COPY apt-dependencies.txt apt-dependencies.txt
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt update && \
    xargs -a apt-dependencies.txt apt install -y --no-install-recommends && \
    yes | unminimize && \
    colcon mixin update && \
    colcon metadata update


# Install newer libignition-math6 (6.15)
RUN \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install libignition-math6-dev -y

# Install dependencies
WORKDIR /home/ubuntu/manipulation/workspace
COPY --from=cacher /tmp/manipulation/workspace/src ./src
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.ros/rosdep,mode=0777,sharing=locked \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro=$ROS_DISTRO \
        --skip-keys slam_toolbox


# Install pip packages
COPY pip-dependencies.txt pip-dependencies.txt
RUN \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    pip3 install -r pip-dependencies.txt

# PEP257 errors are not shown unless you run ament_pep257 on the command line
# see https://github.com/ament/ament_lint/issues/317

# Install neovim
WORKDIR /home/ubuntu
RUN \
    wget https://github.com/neovim/neovim/releases/download/v0.9.4/nvim-linux64.tar.gz && \
    tar -xzvf nvim-linux64.tar.gz && \
    ln -s /home/ubuntu/nvim-linux64/bin/nvim /usr/local/bin/nvim

# Install nodejs
RUN \
    apt install -y ca-certificates gnupg && \
    mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | \
        gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    NODE_MAJOR=20 && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | \
        tee /etc/apt/sources.list.d/nodesource.list && \
    apt update && \
    apt install -y --reinstall nodejs


# Install ble.sh
RUN \
    curl -L https://github.com/akinomyoga/ble.sh/releases/download/v0.4.0-devel3/ble-0.4.0-devel3.tar.xz | tar xJf - -C /home/ubuntu && \
    echo "source /home/ubuntu/manipulation/setup_environment" >> /home/ubuntu/.bashrc && \
    sed -e '/[ -z "$PS1" ] && return/s/^/#/g' -i /home/ubuntu/.bashrc

# Setup completion
RUN curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash \
        -o /home/ubuntu/git-completion.bash

# Give user permission to use chrt
RUN setcap cap_sys_nice+ep $(which chrt)
RUN chmod 755 $(which chrt)

RUN mkdir -p /home/ubuntu/.config
RUN mkdir -p /home/ubuntu/.local
RUN chown -R ubuntu:ubuntu /home/ubuntu/.local /home/ubuntu/.config

# Treat nvim as vim
RUN ln -s /usr/local/bin/nvim /usr/local/bin/vim

# Switch user
USER ubuntu

# Start in this folder
WORKDIR /home/ubuntu/manipulation

ENTRYPOINT ["/home/ubuntu/manipulation/setup_environment"]
CMD ["bash"]
