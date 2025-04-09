FROM ros:noetic

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/root/catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create the catkin workspace
RUN mkdir -p ${ROS_WORKSPACE}/src

# Copy your ROS code into the container
COPY ./src ${ROS_WORKSPACE}/src

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd ${ROS_WORKSPACE} && catkin_make"

# Source the workspace at login
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WORKSPACE}/devel/setup.bash" >> ~/.bashrc

# Working directory
WORKDIR ${ROS_WORKSPACE}

CMD ["bash"]
