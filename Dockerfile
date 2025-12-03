FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV QT_X11_NO_MITSHM=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf \
    ros-noetic-rospy \
    ros-noetic-rosbag \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pillow \
    progressbar2 \
    pykitti \
    numpy \
    pandas \
    jupyterlab \
    ipywidgets

# Install kitti2bag
RUN pip3 install --no-cache-dir kitti2bag

# Create Catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source setup files
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
