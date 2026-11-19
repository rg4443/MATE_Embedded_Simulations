FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg git build-essential cmake pkg-config \
    python3 python3-dev python3-pip python3-future python3-lxml python3-numpy \
    python3-yaml python3-pexpect python3-empy python3-setuptools \
    libtool qtbase5-dev rapidjson-dev cppzmq-dev \
    xvfb x11vnc novnc websockify openbox libgl1-mesa-dri mesa-utils && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic libgz-sim8-dev && \
    rm -rf /var/lib/apt/lists/*
