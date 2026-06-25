FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vida_ws

COPY ros_ws/src ./src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select vida_interfaces

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /vida_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]