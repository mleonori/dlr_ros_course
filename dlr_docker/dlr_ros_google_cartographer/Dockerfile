FROM ros:noetic-ros-core
SHELL ["/bin/bash", "-c"]

# Arguments
# ARG SSH_PRIVATE_KEY
# ARG SSH_PUBLIC_KEY
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Install dependecies and utils
RUN apt-get update -y
# RUN apt-get install -y git openssh-client lsb-core gnupg apt-utils

# Prepare ssh keys and Gitlab
# RUN mkdir -p /root/.ssh
# RUN chmod 700 /root/.ssh

# RUN echo "$SSH_PRIVATE_KEY" > /root/.ssh/id_rsa && \
#     echo "$SSH_PUBLIC_KEY" > /root/.ssh/id_rsa.pub && \
#     chmod 600 /root/.ssh/id_rsa && \
#     chmod 600 /root/.ssh/id_rsa.pub

# RUN eval `ssh-agent -s`
# RUN cat /root/.ssh/id_rsa
# RUN ssh-keyscan gitlab.iit.it > /root/.ssh/known_hosts
# RUN ssh -o StrictHostKeyChecking=no

# Install ROS
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt install curl
# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# RUN apt update
# RUN apt install -y ros-$ROS_DISTRO-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# RUN rosdep init
# RUN rosdep fix-permissions
# RUN rosdep update
# RUN apt install -y ros-$ROS_DISTRO-pinocchio ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-move-base
# RUN apt update
# RUN apt upgrade -y

# Clone and compile libfranka and matlogger2
# RUN git clone git@gitlab.iit.it:hrii/general/hrii_installation_tools.git
# RUN ./hrii_installation_tools/scripts/clone_and_build_matlogger2.sh
# RUN ./hrii_installation_tools/scripts/clone_and_build_libfranka.sh
# RUN ./hrii_installation_tools/scripts/clone_and_build_qbdrivers.sh
# RUN ./hrii_installation_tools/scripts/clone_and_build_unitree_rbdl.sh