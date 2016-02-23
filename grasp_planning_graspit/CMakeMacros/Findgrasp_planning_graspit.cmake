# Try to find grasp_planning_graspit
# 
# You can set the variable
#   GRASP_PLANNING_GRASPIT_PATH
# to point to the installation directory of graspit 
# (the folder which contains the include/ and lib/ dirs)
# in order to allow searching for it there in addition
# to /usr/ and /usr/local/
# 
# Once done this will define
# GRASP_PLANNING_GRASPIT_LIBRARY_FOUND - if graspit is found
# GRASP_PLANNING_GRASPIT_CXXFLAGS - extra flags
# GRASP_PLANNING_GRASPIT_INCLUDE_DIRS - include directories
# GRASP_PLANNING_GRASPIT_LINK_DIRS - link directories
# GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE - the relase version
# GRASP_PLANNING_GRASPIT_LIBRARY_DEBUG - the debug version
# GRASP_PLANNING_GRASPIT_LIBRARY - a default library, with priority debug.
# GRASP_PLANNING_GRASPIT_LIBRARIES - all depending libraries

# --- First, try to find relevant headers with find_path and find_library,
# in order to cause a cmake failure if libraries are not there. 

find_path(GRASP_PLANNING_GRASPIT_PATH grasp_planning_graspit/GraspItSceneManager.h 
	${CMAKE_INCLUDE_PATH}
	/usr/local/include
	/usr/local/grasp_planning_graspit/include
	/usr/include
)

message("Looking for grasp_planning_graspit includes in ${GRASP_PLANNING_GRASPIT_PATH}")

if (GRASP_PLANNING_GRASPIT_PATH)
	message(STATUS "Looking for grasp_planning_graspit headers -- found " ${GRASP_PLANNING_GRASPIT_PATH}/include/grasp_planning_graspit/GraspItSceneManager.h)
    set(GRASP_PLANNING_GRASPIT_PATH_FOUND 1 CACHE INTERNAL "grasp_planning_graspit headers found")
    set(GRASP_PLANNING_GRASPIT_INCLUDE_DIRS 
        ${GRASP_PLANNING_GRASPIT_PATH}/include/ 
        ${GRASP_PLANNING_GRASPIT_PATH}/include/grasp_planning_graspit
    )
else (GRASP_PLANNING_GRASPIT_PATH)
	message(SEND_ERROR 
	"Looking for grasp_planning_graspit headers -- not found"
	"Please install grasp_planning_graspit https://github.com/JenniferBuehler/graspit-pkgs/ or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-grasp-planning-graspit/include ...")
endif (GRASP_PLANNING_GRASPIT_PATH)

find_library(GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE
	NAMES grasp_planning_graspit
	PATHS
	${CMAKE_LIBRARY_PATH}
	${GRASP_PLANNING_GRASPIT_PATH}/lib
	/usr/local/grasp_planning_graspit/lib
	/usr/local/lib
	/usr/lib
    NO_DEFAULT_PATH
)

if (GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE)
	message(STATUS "Looking for grasp_planning_graspit library -- found " ${GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE})
    set (GRASP_PLANNING_GRASPIT_LIBRARY ${GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE})
    # there is no debug library so just use release
    set (GRASP_PLANNING_GRASPIT_LIBRARY_DEBUG ${GRASP_PLANNING_GRASPIT_LIBRARY_DEBUG})
else (GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE)
 	message(SENDL_ERROR 
	"Looking for grasp_planning_graspit library -- not found"
	"Please install grasp_planning_graspit https://github.com/JenniferBuehler/graspit-pkgs/ or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-grasp_planning_graspit/lib ...")
endif (GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE)



# Qt is still required at this stage for using GraspIt!.
# This find_package is to get the headers/libs of Qt required.
# While a separate inclusion of the graspit package may
# already have included the Qt headers, this should be
# done here again, for the case that the graspit package
# is not used directly, but from within a library which
# includes the whole source (in form of a static library).
find_package(Qt4 COMPONENTS QtCore REQUIRED)
set(GRASP_PLANNING_GRASPIT_INCLUDE_DIRS 
    ${GRASP_PLANNING_GRASPIT_INCLUDE_DIRS}
    ${QT_INCLUDES}
) 
set(GRASP_PLANNING_GRASPIT_LIBRARIES 
    ${GRASP_PLANNING_GRASPIT_INCLUDE_DIRS}
    ${GRASP_PLANNING_GRASPIT_LIBRARY}
    ${QT_LIBRARIES}
)

MARK_AS_ADVANCED(
    GRASP_PLANNING_GRASPIT_LIBRARY_FOUND
    GRASP_PLANNING_GRASPIT_CXXFLAGS
    GRASP_PLANNING_GRASPIT_LINK_FLAGS
    GRASP_PLANNING_GRASPIT_INCLUDE_DIRS
    GRASP_PLANNING_GRASPIT_LINK_DIRS
    GRASP_PLANNING_GRASPIT_LIBRARY
    GRASP_PLANNING_GRASPIT_LIBRARY_RELEASE
    GRASP_PLANNING_GRASPIT_LIBRARY_DEBUG
)
