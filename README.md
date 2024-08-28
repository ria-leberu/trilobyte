# trilobyte
Mobile differential drive robot with SLAM and manipulation capabilities. 

Hardware: Raspberry Pi Pico (RP2040), Raspberry Pi 5.

Developed on ROS2 Humble on Ubuntu 22.04.

To install, place into your ros2_ws/src folder.

## Building and Running Docker Container

docker build -t main .

docker run -it --user ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --network=host --ipc=host -m=8g -v /dev:/dev --privileged  main

./launch_dds_agent.sh