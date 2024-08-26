FROM ros:humble-ros-base

# environment variables
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt install -y \
    build-essential \
    python3 \
    git \
    wget \
    python3-rosdep

# init rosdep
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update

# copy the ros package and build it
RUN mkdir -p /ws/src/safety_monitor
RUN git clone https://github.com/FSLART/lart_msgs.git -b dev /ws/src/lart_msgs
COPY . /ws/src/safety_monitor
WORKDIR /ws

# install dependencies
RUN rosdep install --from-paths /ws --ignore-src -r -y

# build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ws && \
    colcon build --parallel-workers 6 --symlink-install"

    #dwdw
# launch the package
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch safety_monitor safetymonitor.launch.xml"]