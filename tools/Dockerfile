FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=4
ARG USER_UID=1000
ARG USER_GID=1000
ARG DQROBOTICS_COMMIT=d4fd2838fe26cccaa4b6e45ad9c7e5e628d10213

# Ubuntu 24.04 / Jazzy: use the system Eigen and compiler consistently.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl ca-certificates sudo locales \
    libeigen3-dev libpoco-dev libfmt-dev libboost-all-dev libyaml-cpp-dev \
    libglfw3-dev libglew-dev libgl1-mesa-dev libosmesa6-dev \
    libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev zlib1g-dev \
    python3-colcon-common-extensions python3-rosdep python3-pip \
    python3-numpy python3-scipy python3-matplotlib python3-pytest \
    xvfb xauth mesa-utils \
    ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-moveit ros-jazzy-warehouse-ros-sqlite ros-jazzy-rosbridge-server \
    ros-jazzy-camera-info-manager ros-jazzy-image-transport \
    ros-jazzy-backward-ros ros-jazzy-generate-parameter-library \
    ros-jazzy-hardware-interface-testing ros-jazzy-ros2-control-test-assets \
    ros-jazzy-ament-cmake-gmock ros-jazzy-ament-cmake-pytest \
    ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
    ros-jazzy-ament-cmake-clang-format ros-jazzy-ament-cmake-clang-tidy \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rqt-controller-manager \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy \
    && locale-gen en_US.UTF-8 zh_CN.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Panda's FCI version must match libfranka; ROS upgrades do not change this pin.
RUN mkdir -p /tmp/libfranka \
    && curl --fail --location --retry 3 --connect-timeout 20 \
      https://codeload.github.com/frankarobotics/libfranka/tar.gz/refs/tags/0.9.2 \
      | tar -xz --strip-components=1 -C /tmp/libfranka \
    && curl --fail --location --retry 3 --connect-timeout 20 \
      https://codeload.github.com/frankaemika/libfranka-common/tar.gz/e6aa0fc210d93fe618bfd8956829a264d5476ba8 \
      | tar -xz --strip-components=1 -C /tmp/libfranka/common \
    && cmake -S /tmp/libfranka -B /tmp/libfranka/build \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/libfranka/build --parallel ${BUILD_JOBS} \
    && cmake --install /tmp/libfranka/build \
    && rm -rf /tmp/libfranka

# The official archive includes headers and shared libraries; the ROS patch
# exports its Findmujoco.cmake module for downstream packages.
# Avoid downloading/building MuJoCo's private third-party dependency trees.
RUN cd /tmp \
    && MUJOCO_ARCH=$(uname -m) \
    && curl --fail --location --retry 3 --connect-timeout 20 --remote-name \
      https://github.com/google-deepmind/mujoco/releases/download/3.2.0/mujoco-3.2.0-linux-${MUJOCO_ARCH}.tar.gz \
    && curl --fail --location --retry 3 --connect-timeout 20 --remote-name \
      https://github.com/google-deepmind/mujoco/releases/download/3.2.0/mujoco-3.2.0-linux-${MUJOCO_ARCH}.tar.gz.sha256 \
    && sha256sum --check mujoco-3.2.0-linux-${MUJOCO_ARCH}.tar.gz.sha256 \
    && tar -xzf mujoco-3.2.0-linux-${MUJOCO_ARCH}.tar.gz --strip-components=1 -C /usr/local \
    && rm mujoco-3.2.0-linux-${MUJOCO_ARCH}.tar.gz*

# Build a fixed DQ Robotics revision against the same Eigen as the workspace.
RUN mkdir /tmp/dqrobotics \
    && curl --fail --location --retry 3 \
      https://codeload.github.com/dqrobotics/cpp/tar.gz/${DQROBOTICS_COMMIT} \
      | tar -xz --strip-components=1 -C /tmp/dqrobotics \
    && cmake -S /tmp/dqrobotics -B /tmp/dqrobotics/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/dqrobotics/build --parallel ${BUILD_JOBS} \
    && cmake --install /tmp/dqrobotics/build \
    && rm -rf /tmp/dqrobotics && ldconfig

RUN groupadd --non-unique --gid ${USER_GID} developer \
    && useradd --non-unique --uid ${USER_UID} --gid developer \
      --create-home --shell /bin/bash developer \
    && echo 'developer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/developer \
    && chmod 0440 /etc/sudoers.d/developer \
    && mkdir -p /home/developer/multipanda_ws/src \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/developer/.bashrc \
    && echo 'if [ -f ~/multipanda_ws/install/setup.bash ]; then source ~/multipanda_ws/install/setup.bash; fi' >> /home/developer/.bashrc \
    && chown -R ${USER_UID}:${USER_GID} /home/developer

ENV LANG=zh_CN.UTF-8
ENV LC_ALL=zh_CN.UTF-8
USER developer
WORKDIR /home/developer/multipanda_ws
CMD ["bash"]
