cmake_minimum_required(VERSION 2.8.3)
project(urdf2graspit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  ivcon
  eigen_conversions
  roslint
)

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")
find_package(SoQt4 REQUIRED)
find_package(Qt4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urdf2graspit
  CATKIN_DEPENDS ivcon roscpp urdf eigen_conversions 
  DEPENDS Qt4 SoQt4 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SOQt_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/Qt
)


## Declare a cpp library
add_library(urdf2graspit
    src/Urdf2Graspit.cpp
    src/DHParam.cpp
    src/Helpers.cpp
    src/MarkerSelector.cpp
    src/XMLFuncs.cpp
    src/FileIO.cpp
    src/OutputStructure.cpp
)

## Declare a cpp executable
add_executable(urdf2graspit_node src/urdf2graspit_node.cpp)
add_executable(urdf_viewer src/viewer.cpp src/MarkerSelector.cpp src/XMLFuncs.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(urdf2graspit_node urdf2graspit_generate_messages_cpp)

target_link_libraries(urdf2graspit
   ${catkin_LIBRARIES}
)

target_link_libraries(urdf2graspit_node
   ${catkin_LIBRARIES}
    urdf2graspit
)

target_link_libraries(urdf_viewer
   ${catkin_LIBRARIES}
   ${SOQT_LIBRARY}
   ${QT_LIBRARIES}
)

# style checking
roslint_cpp()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS urdf2graspit urdf2graspit_node urdf_viewer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_urdf2graspit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
