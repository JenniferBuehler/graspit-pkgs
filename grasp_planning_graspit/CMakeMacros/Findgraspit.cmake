# Try to find graspit
# 
# You can set the variable
#   GRASPIT_PATH
# to point to the installation directory of graspit 
# (the folder which contains the include/ and lib/ dirs)
# in order to allow searching for it there in addition
# to /usr/ and /usr/local/
# 
# Once done this will define
# GRASPIT_LIBRARY_FOUND - if graspit is found
# GRASPIT_CXXFLAGS - extra flags
# GRASPIT_INCLUDE_DIRS - include directories
# GRASPIT_LINK_DIRS - link directories
# GRASPIT_LIBRARY_RELEASE - the relase version
# GRASPIT_LIBRARY_DEBUG - the debug version
# GRASPIT_LIBRARY - a default library, with priority debug.

# --- First, try to find relevant headers with find_path and find_library,
# in order to cause a cmake failure if libraries are not there. 

find_path(GRASPIT_PATH ivmgr.h 
	${CMAKE_INCLUDE_PATH}
	/usr/local/include
	/usr/local/graspit/include
	/usr/include
)

message("Looking for graspit includes in ${GRASPIT_PATH}")

if (GRASPIT_PATH)
	message(STATUS "Looking for graspit headers -- found " ${GRASPIT_PATH}/include/ivmgr.h)
    set(GRASPIT_PATH_FOUND 1 CACHE INTERNAL "graspit headers found")
    set(GRASPIT_INCLUDE_DIRS 
        ${GRASPIT_PATH}/include/ 
        ${GRASPIT_PATH}/include/graspit 
    )
else (GRASPIT_PATH)
	message(SEND_ERROR 
	"Looking for graspit headers -- not found"
	"Please install graspit https://github.com/graspit-simulator/graspit or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_INCLUDE_PATH=/path-to-graspit/include ...")
endif (GRASPIT_PATH)

find_library(GRASPIT_LIBRARY_RELEASE
	NAMES graspit
	PATHS
	${CMAKE_LIBRARY_PATH}
	${GRASPIT_PATH}/lib
	/usr/local/graspit/lib
	/usr/local/lib
	/usr/lib
    NO_DEFAULT_PATH
)

if (GRASPIT_LIBRARY_RELEASE)
	message(STATUS "Looking for graspit library -- found " ${GRASPIT_LIBRARY_RELEASE})
    set (GRASPIT_LIBRARY ${GRASPIT_LIBRARY_RELEASE})
    # there is no debug version yet
    set (GRASPIT_LIBRARY_DEBUG ${GRASPIT_LIBRARY_RELEASE})
else (GRASPIT_LIBRARY_RELEASE)
 	message(SENDL_ERROR 
	"Looking for graspit library -- not found"
	"Please install graspit https://github.com/graspit-simulator/graspit or adjust CMAKE_INCLUDE_PATH"
	"e.g. cmake -DCMAKE_LIBRARY_PATH=/path-to-graspit/lib ...")
endif (GRASPIT_LIBRARY_RELEASE)

MARK_AS_ADVANCED(
    GRASPIT_LIBRARY_FOUND
    GRASPIT_CXXFLAGS
    GRASPIT_LINK_FLAGS
    GRASPIT_INCLUDE_DIRS
    GRASPIT_LINK_DIRS
    GRASPIT_LIBRARY
    GRASPIT_LIBRARY_RELEASE
    GRASPIT_LIBRARY_DEBUG
)
