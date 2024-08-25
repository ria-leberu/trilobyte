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
    && colcon build \
    && . /opt/ros/humble/setup.sh



# Copy project into docker ROS2 workspace


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc



ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

COPY trilobyte/ /home/${USERNAME}/ros2_ws/src

# RUN cd home/${USERNAME}/ros2_ws \
#     && colcon build