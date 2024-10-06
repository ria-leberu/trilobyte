FROM ros:humble

# installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    screen \
    python3-pip \
    ros-humble-demo-nodes-cpp \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ament-cmake-clang-format \
    ros-dev-tools \
    ros-humble-ament-* \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./

RUN pip install --no-cache-dir --upgrade pip \
  && pip install --no-cache-dir -r requirements.txt

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Make ROS2 workspace
RUN mkdir -p home/${USERNAME}/trilobyte_ws/src

COPY trilobyte_base/ /home/${USERNAME}/trilobyte_ws/src/trilobyte_base/
COPY trilobyte_description/ /home/${USERNAME}/trilobyte_ws/src/trilobyte_description/
COPY trilobyte_hardware_interface/ /home/${USERNAME}/trilobyte_ws/src/trilobyte_hardware_interface/
COPY trilobyte/ /home/${USERNAME}/trilobyte_ws/src/trilobyte/

COPY ldlidar_stl_ros2/ /home/${USERNAME}/trilobyte_ws/src/ldlidar_stl_ros2/

RUN cd home/${USERNAME}/trilobyte_ws \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build

RUN usermod -aG dialout ${USERNAME}
# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
