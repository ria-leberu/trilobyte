FROM ros:humble

# installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    ros-humble-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

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
RUN mkdir -p home/${USERNAME}/ros2_ws/src

RUN cd home/${USERNAME}/ros2_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build

COPY trilobyte_base/ /home/${USERNAME}/ros2_ws/src/trilobyte/trilobyte_base/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd home/ros/ros2_ws && colcon build && source install/setup.bash"

# Install MicroXRCE DDS Agent
RUN /bin/bash -c "cd home/${USERNAME}/ros2_ws/src/trilobyte/trilobyte_base/dds_agent/Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && cmake .. && make && make install && sudo ldconfig /usr/local/lib"

COPY launch_dds_agent.sh /launch_dds_agent.sh 

RUN usermod -aG dialout ${USERNAME}
# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
