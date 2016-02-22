cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)

set (CMAKE_MODULE_PATH "${GRASP_PLANNING_GRASPIT_PATH}/share/CMakeMacros/")

find_package(Eigen3 REQUIRED)
find_package (graspit REQUIRED)
find_package (grasp_planning_graspit REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)

find_package(Qhull REQUIRED)
find_package(SoQt4 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Threads REQUIRED)

SET( QT_USE_QT3SUPPORT TRUE )
find_package(Qt4 COMPONENTS QtCore REQUIRED)

# include QT_USE_FILE is needed.
# include (${QT_USE_FILE})

###########
## Build ##
###########
# ADD_DEFINITIONS(
#    -std=c++11 
#)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${SOQT_INCLUDE_DIRS}
  ${QT_INCLUDES}
  ${QT_INCLUDE_DIR}
  ${QT_QT3SUPPORT_INCLUDE_DIR}
  ${QHULL_INCLUDE_DIRS}
  ${GRASPIT_INCLUDE_DIRS}
  ${GRASP_PLANNING_GRASPIT_INCLUDE_DIRS}
)

add_executable(simple_planning_tutorial src/simple_planning_tutorial.cpp)

target_link_libraries(simple_planning_tutorial
   ${Boost_LIBRARIES}
   ${QT_LIBRARIES}
   ${QT_QT3SUPPORT_LIBRARY}
   ${QHULL_LIBRARIES}
   ${SOQT_LIBRARY}
   ${LAPACK_LIBRARIES}
   ${GRASPIT_LIBRARY}
   ${GRASP_PLANNING_GRASPIT_LIBRARY}
)
