cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)

set (CMAKE_MODULE_PATH "${GRASP_PLANNING_GRASPIT_PATH}/share/CMakeMacros/")

# if a path to graspit is specified, add this library separately:
if (GRASPIT_PATH)
    find_package (graspit REQUIRED)
endif (GRASPIT_PATH)

find_package (grasp_planning_graspit REQUIRED)

###########
## Build ##
###########
# ADD_DEFINITIONS(
#    -std=c++11 
#)

include_directories(
    ${grasp_planning_graspit_INCLUDE_DIRS}
)

add_executable(simple_planning_tutorial src/simple_planning_tutorial.cpp)

target_link_libraries(simple_planning_tutorial
   ${grasp_planning_graspit_LIBRARIES}
)
