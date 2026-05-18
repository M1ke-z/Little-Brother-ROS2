FROM ros:jazzy-ros-base

#install joystick packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    vim \
&& rm -rf /var/lib/apt/lists/*

# Auto-source ROS
RUN echo "source install/setup.bash" >> /root/.bashrc

WORKDIR /ws

CMD ["bash"]
