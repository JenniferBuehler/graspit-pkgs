cmake_minimum_required(VERSION 2.8.3)
project(urdf2graspit)

set (CATKIN_PKGS
  roscpp
  urdf
  eigen_conversions
  baselib_binding
  urdf2inventor
  urdf_viewer
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS} roslint)

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
  CATKIN_DEPENDS ${CATKIN_PKGS} 
  DEPENDS git
)

###########
## Build ##
###########

add_definitions(${baselib_binding_DEFINITIONS} ${urdf2inventor_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(urdf2graspit
    src/Urdf2GraspItBase.cpp
    src/Urdf2GraspIt.cpp
    src/ContactFunctions.cpp
    src/ContactsGenerator.cpp
    src/DHParam.cpp
    src/MarkerSelector.cpp
    src/XMLFuncs.cpp
    src/FileIO.cpp
    src/OutputStructure.cpp
    src/ConvertGraspitMesh.cpp
)

## Declare a cpp executable
add_executable(urdf2graspit_node src/urdf2graspit_node.cpp)
add_executable(contacts_generator_node src/contacts_generator_node.cpp)
add_executable(marker_selector_viewer src/viewer.cpp src/MarkerSelector.cpp src/XMLFuncs.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(urdf2graspit_node urdf2graspit_generate_messages_cpp)

add_dependencies(urdf2graspit_node urdf2graspit)
add_dependencies(contacts_generator_node urdf2graspit)

target_link_libraries(urdf2graspit ${catkin_LIBRARIES})
target_link_libraries(urdf2graspit_node urdf2graspit ${catkin_LIBRARIES})
target_link_libraries(contacts_generator_node urdf2graspit ${catkin_LIBRARIES})
target_link_libraries(marker_selector_viewer ${catkin_LIBRARIES})

# style checking
roslint_cpp()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS urdf2graspit urdf2graspit_node contacts_generator_node marker_selector_viewer
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

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
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
