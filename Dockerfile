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

FROM jenniferbuehler/graspit_autobuild_ros 

MAINTAINER Jennifer Buehler

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-indigo-eigen-conversions \
    ros-indigo-household-objects-database \
    ros-indigo-household-objects-database-msgs  \
    ros-indigo-manipulation-msgs \
    ros-indigo-object-recognition-msgs \
    ros-indigo-roslint \
    && rm -rf /var/lib/apt/lists/

COPY grasp_planning_graspit_ros /catkin_ws/src/grasp_planning_graspit_ros
COPY grasp_planning_graspit /catkin_ws/src/grasp_planning_graspit
COPY grasp_planning_graspit_msgs /catkin_ws/src/grasp_planning_graspit_msgs
COPY urdf2graspit /catkin_ws/src/urdf2graspit

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
