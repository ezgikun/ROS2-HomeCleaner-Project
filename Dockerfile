FROM osrf/ros:humble-desktop-full

LABEL maintainer="HomeCleanerBot Team"

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-gazebo \
    ros-humble-xacro \
    python3-pip \
    nano \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/.gazebo/models

WORKDIR /root/homecleaner_ws
COPY src ./src

RUN cp -r src/homecleaner_bot/world/my_home_layout /root/.gazebo/models/ || true

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/homecleaner_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
