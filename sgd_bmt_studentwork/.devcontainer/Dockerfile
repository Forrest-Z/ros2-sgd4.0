# https://hub.docker.com/layers/library/ros/foxy/images/sha256-5991759125bfb3b5f7b99ea8d89519c620973f2fafe9f578b06683a187dd4b90
FROM ros:foxy

ARG USERNAME=bmt6
ARG USER_UID=1000
ARG USER_GID=$USER_UID

WORKDIR /home/ws

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip wget

## Install needed libraries
RUN pip3 install paho-mqtt

# source /opt/ros/$ROS_DISTRO/setup.bash
## It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
# rosdep update
# rosdep install -i --from-path src --rosdistro foxy -y
## Still in the root of your workspace, ros2_ws, build your new package:
# colcon build --packages-select py_pubsub

COPY ros2 .

RUN rosdep update \ 
    && rosdep install -i --from-path src --rosdistro foxy -y \
    && colcon build --packages-select py_srvcli py_pubsub

# build the project
RUN colcon build

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Setup enviroment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

ENV SHELL /bin/bash
CMD [ "bash" ]
