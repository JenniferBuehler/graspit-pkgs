cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)

find_package (grasp_planning_graspit REQUIRED)

include_directories(
    ${grasp_planning_graspit_INCLUDE_DIRS}
)

add_executable(simple_planning_tutorial src/simple_planning_tutorial.cpp)

target_link_libraries(simple_planning_tutorial
   ${grasp_planning_graspit_LIBRARIES}
)
