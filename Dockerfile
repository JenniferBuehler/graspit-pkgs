# In order to support graphical interfaces,
# this should be run with 
# docker run -it --rm \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \  
#     ros-indigo-full-catkin <cmd> 
#
# The -e and -v commands are needed to display on the host X server.
# For hardware support, you will also need:
#  --privileged   (to access the graphics card) 
#  It may also be required to call
#  $ xhost +
#  before running the container.

FROM jenniferbuehler/ros-indigo-full-catkin 

MAINTAINER Jennifer Buehler

# Install system essentials
RUN apt-get update && apt-get install -y \
    cmake \
    libsoqt4-dev \
    libcoin80-dev \
    libqt4-dev \
    libblas-dev \
    liblapack-dev \
    libqhull-dev \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# need g++ for compiling with cmake even if gcc
# is already installed
RUN apt-get update && apt-get install -y g++ \
    && rm -rf /var/lib/apt/lists/*

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-indigo-eigen-conversions \
    ros-indigo-household-objects-database \
    ros-indigo-household-objects-database-msgs  \
    ros-indigo-manipulation-msgs \
    ros-indigo-object-recognition-msgs \
    ros-indigo-roslint \
    && rm -rf /var/lib/apt/lists/

COPY graspit_tools /catkin_ws/src/graspit_tools
COPY grasp_planning_graspit_ros /catkin_ws/src/grasp_planning_graspit_ros
COPY grasp_planning_graspit /catkin_ws/src/grasp_planning_graspit
COPY grasp_planning_graspit_msgs /catkin_ws/src/grasp_planning_graspit_msgs
COPY urdf2graspit /catkin_ws/src/urdf2graspit
COPY jaco_graspit_sample /catkin_ws/src/jaco_graspit_sample

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

# set GRASPIT environment to /graspit_home
RUN bin/bash -c "mkdir -p /graspit_home/models/robots"
RUN bin/bash -c "mkdir -p /graspit_home/models/objects"
RUN bin/bash -c "mkdir -p /graspit_home/worlds"
ENV GRASPIT /graspit_home

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
