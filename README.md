# trilobyte
Mobile differential drive robot with SLAM and manipulation capabilities. 

Hardware: Raspberry Pi Pico (RP2040), Raspberry Pi 5.

Developed on ROS2 Humble on Ubuntu 22.04.

To install, place into your ros2_ws/src folder.

## Building and Running Docker Container



### Raspberry Pi 5 

    docker build -f Dockerfile.rpi -t rpi .
    docker run -it --user ros --network=host --ipc=host --privileged rpi

__Commands__  

    ros2 launch trilobyte_bringup trilobyte.launch.py


### Local PC 

    docker build -f Dockerfile.local -t local .
    docker run -it --user ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /home/$USER/trilobyte/trilobyte_description/rviz:/home/$USER/trilobyte/trilobyte_description/rviz --env=DISPLAY --network=host --ipc=host local







