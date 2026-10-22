FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-serial \
    libeigen3-dev \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-diagnostic-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-rosidl-default-generators \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vida_ws

COPY src ./src

RUN source /opt/ros/humble/setup.bash && \
    colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /vida_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
