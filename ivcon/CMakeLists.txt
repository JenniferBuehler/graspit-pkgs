cmake_minimum_required(VERSION 2.8.3)
project(ivcon)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")
find_package(Qt4 REQUIRED)
find_package(Coin REQUIRED)
find_package(SoQt4 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  INCLUDE_DIRS include include/ivcon
  LIBRARIES ivcon
  CATKIN_DEPENDS roscpp
  DEPENDS Qt4 Coin SoQt4
)

###########
## Build ##
###########

ADD_DEFINITIONS(${QT_DEFINITIONS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/ivcon
  include
  ${SoQt_INCLUDE_DIR}
  ${Coin_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/Qt
)

## Declare a cpp library
add_library(ivcon
  src/asc.cpp  src/dxf.cpp   src/ivconv.cpp  src/mesh.cpp  src/smf.cpp  src/tec.cpp   src/txt.cpp    src/vla.cpp
src/ase.cpp  src/gmod.cpp  src/iv.cpp     src/obj.cpp   src/stl.cpp  src/tmat.cpp  src/ucd.cpp    src/wrl.cpp
src/byu.cpp  src/hrc.cpp   src/pov.cpp   src/tds.cpp  src/tri.cpp  src/utils.cpp  src/xgl.cpp
)

## Declare a cpp executable
add_executable(ivcon_node src/main.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ivcon_node ivcon_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ivcon
   ${catkin_LIBRARIES}
   ${SOQT_LIBRARY}
   ${Coin_LIBRARIES}
   ${QT_LIBRARIES}
)
target_link_libraries(ivcon_node
   ivcon 
   ${catkin_LIBRARIES}
   ${SOQT_LIBRARY}
   ${Coin_LIBRARIES}
   ${QT_LIBRARIES}
)


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
install(TARGETS ivcon ivcon_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ivcon.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
