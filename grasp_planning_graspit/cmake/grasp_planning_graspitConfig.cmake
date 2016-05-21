# This will define
# grasp_planning_graspit_CXXFLAGS - extra flags
# grasp_planning_graspit_INCLUDE_DIRS - include directories
# grasp_planning_graspit_LINK_DIRS - link directories
# grasp_planning_graspit_LIBRARY_RELEASE - the relase version
# grasp_planning_graspit_LIBRARY_DEBUG - the debug version
# grasp_planning_graspit_LIBRARY - a default library, with priority debug.
# grasp_planning_graspit_LIBRARIES - all depending libraries

# When using catkin, the include dirs are determined by catkin itself
if (NOT CATKIN_DEVEL_PREFIX)
    get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
    # message("$$$$$$$ ${CMAKE_CURRENT_LIST_FILE} SELF_DIR = ${SELF_DIR}")
    get_filename_component(grasp_planning_graspit_INCLUDE_DIR "${SELF_DIR}/../../include/" ABSOLUTE)
    get_filename_component(grasp_planning_graspit_LINK_DIR "${SELF_DIR}/../../lib/" ABSOLUTE)

    if (grasp_planning_graspit_INCLUDE_DIR)
        message(STATUS "Looking for grasp_planning_graspit headers -- found " ${grasp_planning_graspit_INCLUDE_DIR})
        set(grasp_planning_graspit_INCLUDE_DIRS 
            ${grasp_planning_graspit_INCLUDE_DIR} 
            ${grasp_planning_graspit_INCLUDE_DIR}/grasp_planning_graspit 
        )
    else (grasp_planning_graspit_INCLUDE_DIR)
        message(SEND_ERROR 
        "Looking for grasp_planning_graspit headers -- not found"
        "Please install grasp_planning_graspit or adjust CMAKE_PREFIX_PATH"
        "e.g. cmake -DCMAKE_PREFIX_PATH=/path-to-grasp_planning_graspit/ ...")
    endif (grasp_planning_graspit_INCLUDE_DIR)

    if (grasp_planning_graspit_LINK_DIR)
        message(STATUS "Looking for grasp_planning_graspit library include -- found " ${grasp_planning_graspit_LINK_DIR})
        set(grasp_planning_graspit_LINK_DIRS 
            ${grasp_planning_graspit_LINK_DIR}/lib/ 
            ${grasp_planning_graspit_LINK_DIR}/lib/grasp_planning_graspit 
        )
    else (grasp_planning_graspit_LINK_DIR)
        message(SEND_ERROR 
        "Looking for grasp_planning_graspit library directories -- not found"
        "Please install grasp_planning_graspit or adjust CMAKE_PREFIX_PATH"
        "e.g. cmake -DCMAKE_PREVIX_PATH=/path-to-grasp_planning_graspit/ ...")
    endif (grasp_planning_graspit_LINK_DIR)

    include(${SELF_DIR}/grasp_planning_graspit-targets.cmake)
    set (CMAKE_MODULE_PATH ${SELF_DIR})
else (NOT CATKIN_DEVEL_PREFIX)
    # catkin is going to handle the include directories
    include(${CATKIN_DEVEL_PREFIX}/lib/grasp_planning_graspit/grasp_planning_graspit-targets.cmake)
endif (NOT CATKIN_DEVEL_PREFIX)

find_library(grasp_planning_graspit_LIBRARY_RELEASE
	NAMES grasp_planning_graspit
	PATHS
	${CMAKE_LIBRARY_PATH}
    ${grasp_planning_graspit_LINK_DIR}
	/usr/local/grasp_planning_graspit/lib
	/usr/local/lib
	/usr/lib
    NO_DEFAULT_PATH
)

if (grasp_planning_graspit_LIBRARY_RELEASE)
	message(STATUS "Looking for grasp_planning_graspit library -- found " ${grasp_planning_graspit_LIBRARY_RELEASE})
    set (grasp_planning_graspit_LIBRARY ${grasp_planning_graspit_LIBRARY_RELEASE})
    # there is no debug library so just use release
    set (grasp_planning_graspit_LIBRARY_DEBUG ${grasp_planning_graspit_LIBRARY_DEBUG})
else (grasp_planning_graspit_LIBRARY_RELEASE)
 	message(SENDL_ERROR 
	"Looking for grasp_planning_graspit library -- not found"
    "Please install grasp_planning_graspit or adjust CMAKE_PREFIX_PATH"
    "e.g. cmake -DCMAKE_PREFIX_PATH=/path-to-grasp_planning_graspit/lib ...")
endif (grasp_planning_graspit_LIBRARY_RELEASE)

find_package(graspit REQUIRED)

# when building without catkin, Eigen3 has to be found separately
# (otherwise it is included with eigen_conversions)
if (NOT CATKIN_DEVEL_PREFIX)
    find_package(Eigen3 REQUIRED)
endif (NOT CATKIN_DEVEL_PREFIX)

find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)

set(grasp_planning_graspit_INCLUDE_DIRS 
    ${grasp_planning_graspit_INCLUDE_DIRS}
    ${graspit_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
) 
set(grasp_planning_graspit_LIBRARIES 
    ${grasp_planning_graspit_LIBRARY}
    ${graspit_LIBRARIES}
    ${Boost_LIBRARIES}
)

set(grasp_planning_graspit_CXXFLAGS ${graspit_CXXFLAGS})

MARK_AS_ADVANCED(
    grasp_planning_graspit_LIBRARY_FOUND
    grasp_planning_graspit_CXXFLAGS
    grasp_planning_graspit_LINK_FLAGS
    grasp_planning_graspit_INCLUDE_DIRS
    grasp_planning_graspit_LINK_DIRS
    grasp_planning_graspit_LIBRARY
    grasp_planning_graspit_LIBRARY_RELEASE
    grasp_planning_graspit_LIBRARY_DEBUG
    grasp_planning_graspit_LIBRARIES
)
