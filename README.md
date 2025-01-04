# trilobyte
Mobile differential drive robot with SLAM and manipulation capabilities. 

Hardware: Raspberry Pi Pico (RP2040), Raspberry Pi 5.

Developed on ROS2 Humble on Ubuntu 22.04.

Install Hailo on Pi
Configure Raspi-Config Interface Options, No to Serial Console, Yes to Serial Hardware

## Building and Running Docker Container



### Raspberry Pi 5 

    docker build -f Dockerfile.rpi -t rpi .
    docker run -it --user ros --network=host --ipc=host --privileged rpi

__Commands__  

    ros2 launch trilobyte_bringup trilobyte.launch.py


### Local PC 

    docker build -f Dockerfile.local -t local .
    docker run -it --user ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /home/$USER/trilobyte/trilobyte_description/rviz:/home/$USER/trilobyte/trilobyte_description/rviz --env=DISPLAY --network=host --ipc=host local

__Commands__

    ros2 launch trilobyte_description rviz.launch.py


__Works but color not showing__

ros2 run realsense2_camera realsense2_camera_node --ros-args -p pointcloud.enable:=true



ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth:=true