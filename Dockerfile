FROM osrf/ros:humble-desktop-full

RUN sudo apt update -y
RUN sudo apt install -y ros-humble-tf-transformations ros-humble-slam-toolbox\
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs\
    ros-humble-rmw-cyclonedds-cpp gdb

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /workspaces/gh_mk2/install/setup.bash' >> /root/.bashrc