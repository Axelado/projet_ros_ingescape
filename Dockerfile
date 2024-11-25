FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
# run a bash on startup by passing "bash" argument to entrypoint.sh (check @ in entrypoint)
CMD ["bash"] 

USER ros
# everything i type here it's own by user ros

USER root
# everything i type here it's own by user root
COPY pizibot_gazebo/ /ros2_ws/src/pizibot_gazebo/
COPY pizibot_description/ /ros2_ws/src/pizibot_description/
COPY pizibot_teleop/ /ros2_ws/src/pizibot_teleop/
COPY pizibot_navigation/ /ros2_ws/src/pizibot_navigation/
COPY ingescape_ros2_interface/ /ros2_ws/src/ingescape_ros2_interface/


RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    joystick \
    jstest-gtk \
    evtest \
    ros-humble-twist-mux \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-opencv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


RUN pip install ingescape


# RUN mkdir -p /ros2_ws/src

# Définir le workspace comme le répertoire de travail courant
WORKDIR /ros2_ws
# RUN git clone https://github.com/Axelado/pizibot.git src/pizibot


# Installer les dépendances du package
RUN apt-get update

USER ros
RUN rosdep update \ 
    && rosdep install --from-paths src --ignore-src -r -y

USER root
RUN rm -rf /var/lib/apt/lists/*

# Compiler le workspace
RUN . /opt/ros/humble/setup.sh && colcon build --packages-skip pizibot_hardware


# Source l'overlay du workspace
# RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

#the last user define in the dockerfile will be the default user on container start-up (but it can be override by --user )

WORKDIR /home/$USERNAME/

ENV DEBIAN_FRONTEND=