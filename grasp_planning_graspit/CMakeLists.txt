cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)
    
# by default, compiling with catkin is
# enabled. It will only be disabled again
# if CATKIN_DEVEL_PREFIX is not set.
# To enforce compiling *without* catkin
# even if CATKIN_DEVEL_PREFIX is defined,
# compile with the flag -COMPILE_WITH_CATKIN=false

# To enforce compiling with catkin in ANY case,
# set ENFORCE_CATKIN to true. This is for example
# required when compiling with jenkins, when
# it seems that CATKIN_DEVEL_PREFIX is not set.
# So set ENFORCE_CATKIN to true only for the commits
# to jenkins.

set (ENFORCE_CATKIN true)
if (ENFORCE_CATKIN)
    message(STATUS "Enforcing compilation with catkin, as ENFORCE_CATKIN was set to true")
    set (COMPILE_WITH_CATKIN true)
endif (ENFORCE_CATKIN)

if (NOT DEFINED COMPILE_WITH_CATKIN)
    message(STATUS "Compiling grasp_planning_graspit: COMPILE_WITH_CATKIN is not defined, setting to TRUE by default and **compiling with catkin**.")
    set (COMPILE_WITH_CATKIN true)
    set (COMPILE_WITH_CATKIN_DEFAULT true) # flag that compiling with catkin is due to default setting
endif (NOT DEFINED COMPILE_WITH_CATKIN)

if (COMPILE_WITH_CATKIN AND NOT DEFINED COMPILE_WITH_CATKIN_DEFAULT)
    message(STATUS "Compiling grasp_planning_graspit: COMPILE_WITH_CATKIN is true, so **compiling with catkin**.")
endif (COMPILE_WITH_CATKIN AND NOT DEFINED COMPILE_WITH_CATKIN_DEFAULT)

if (NOT COMPILE_WITH_CATKIN)
    message(STATUS "Compiling grasp_planning_graspit: COMPILE_WITH_CATKIN is false, so **compiling with cmake.**")
endif (NOT COMPILE_WITH_CATKIN)

if (NOT ENFORCE_CATKIN AND COMPILE_WITH_CATKIN AND NOT CATKIN_DEVEL_PREFIX)
    set(MSG_STATUS STATUS)
    if (NOT COMPILE_WITH_CATKIN_DEFAULT)
        set(MSG_STATUS WARNING)
    endif (NOT COMPILE_WITH_CATKIN_DEFAULT)
    message(${MSG_STATUS} "Compiling grasp_planning_graspit: COMPILE_WITH_CATKIN is true, but CATKIN_DEVEL_PREFIX is not defined, so **enforcing compilation with cmake**.")
    set (COMPILE_WITH_CATKIN false)
endif (NOT ENFORCE_CATKIN AND COMPILE_WITH_CATKIN AND NOT CATKIN_DEVEL_PREFIX)


# set this variable to true if you would like that cmake
# downloads and builds the fork of GraspIt! which
# is required for this package (https://github.com/JenniferBuehler/graspit).
# If this is set to false, it will search for the package
# GraspIt! using CMakeMacros/Findgraspit.cmake, for which you
# may set the variable GRASPIT_PATH to the directory where
# you installed the graspit fork.
# IMPORTANT: If you set this to false and are compiling
# with catkin, uncomment the <*_depend> tags relating to
# graspit in package.xml!
if (NOT DEFINED BUILD_GRASPIT)
    set (BUILD_GRASPIT true)
endif (NOT DEFINED BUILD_GRASPIT)

set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)

if (COMPILE_WITH_CATKIN)
    find_package(catkin REQUIRED COMPONENTS eigen_conversions roslint) 
    if (NOT BUILD_GRASPIT)
        find_package(catkin REQUIRED COMPONENTS graspit) 
    endif (NOT BUILD_GRASPIT)
    # include QT_USE_FILE is not necessary, because it is included via the catkin dependencies.
endif (COMPILE_WITH_CATKIN) 

# if catkin is not used (or graspit built locally),
# we have to get the Qt dependencies separately
if (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)
    if (NOT BUILD_GRASPIT)
        find_package (graspit REQUIRED)
    endif (NOT BUILD_GRASPIT)

    find_package(Qhull REQUIRED)
    find_package(SoQt4 REQUIRED)
    find_package(LAPACK REQUIRED)
    find_package(Threads REQUIRED)

    SET( QT_USE_QT3SUPPORT TRUE )
    find_package(Qt4 COMPONENTS QtCore REQUIRED)

    # include QT_USE_FILE is needed.
    include (${QT_USE_FILE})

    find_package(Eigen3 REQUIRED)
endif (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)


set(GRASPIT_SYSTEM_LIBS
   ${QT_LIBRARIES}
   ${QT_QT3SUPPORT_LIBRARY}
   ${QHULL_LIBRARIES}
   ${SOQT_LIBRARY}
   ${LAPACK_LIBRARIES}
)


# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    # can check with if (TARGET doc) whether the target 'doc' was already used for another package.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(grasp_planning_graspit_doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)

# if the graspit fork should be build with this package,
# do this now.
if (BUILD_GRASPIT)
    set (GLOBAL_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/graspit/install)
    include(CMakeLists-external-graspit.txt)
endif (BUILD_GRASPIT)


# Qt MOC files are needed ONLY because QT slots are used at the moment in EigenGraspPlanner.h
# Qt include files should be covered by the implicit dependency of graspit libraries
# if compiled with catkin
set (MOC_HEADERS include/grasp_planning_graspit/EigenGraspPlanner.h) 
QT4_WRAP_CPP(MOC_OUTFILES ${MOC_HEADERS})

     
if (COMPILE_WITH_CATKIN)
    # INCLUDE_DIRS for catkin_package() command
    set(CATKIN_DEPEND_INCLUDE_DIRS
        include
        ${QT_INCLUDES}
    )    
    
    # CATKIN_DEPENDS for catkin_package() command
    set (CATKIN_DEPEND_PKGS eigen_conversions)
    if (NOT BUILD_GRASPIT)
        set (CATKIN_DEPEND_PKGS ${CATKIN_DEPEND_PKGS} graspit)
    endif (NOT BUILD_GRASPIT)

    # DEPENDS for catkin_package() command
    #set(CATKIN_DEPENDS_LIBS ${Boost_LIBRARIES})
    #if (BUILD_GRASPIT)
    #    set(CATKIN_DEPENDS_LIBS
    #        ${CATKIN_DEPENDS_LIBS}
    #        ${GRASPIT_SYSTEM_LIBS} 
    #    )
    #endif (BUILD_GRASPIT)

    set(CATKIN_DEPENDS_LIBS boost libsoqt4-dev libcoin80-dev libqt4-dev libblas-dev liblapack-dev libqhull)

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
      INCLUDE_DIRS ${CATKIN_DEPEND_INCLUDE_DIRS}
      LIBRARIES grasp_planning_graspit
      CATKIN_DEPENDS ${CATKIN_DEPEND_PKGS} 
      DEPENDS ${CATKIN_DEPENDS_LIBS}
    )
endif (COMPILE_WITH_CATKIN)


###########
## Build ##
###########

set(ALL_INCLUDE_DIRECTORIES
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if (COMPILE_WITH_CATKIN)
    set(ALL_INCLUDE_DIRECTORIES ${ALL_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
endif (COMPILE_WITH_CATKIN)

if (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)
    set(ALL_INCLUDE_DIRECTORIES
         ${ALL_INCLUDE_DIRECTORIES}
         ${GRASPIT_INCLUDE_DIRS}
         ${SOQT_INCLUDE_DIRS}
         ${QT_INCLUDES}
         ${QT_INCLUDE_DIR}
         ${QT_QT3SUPPORT_INCLUDE_DIR}
         ${QHULL_INCLUDE_DIRS}
    )
endif (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)

#ADD_DEFINITIONS(
    # -std=c++11 
    # -g
    # -DGRASPITDBG
#)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${ALL_INCLUDE_DIRECTORIES}
)

set (INCDIR include/${PROJECT_NAME})
set(MAIN_HEADER_FILES
    ${INCDIR}/GraspItSceneManager.h
    ${INCDIR}/GraspItSceneManagerNoGui.h
    ${INCDIR}/GraspItAccessor.h
    ${INCDIR}/GraspItDatabaseManager.h
    ${INCDIR}/GraspItSimpleDBManager.h
    ${INCDIR}/EigenGraspPlanner.h
    ${INCDIR}/EigenGraspPlannerNoQt.h
    ${INCDIR}/EigenGraspResult.h
    ${INCDIR}/GraspItTypes.h
    ${INCDIR}/SharedPtr.h
    ${INCDIR}/ThreadImpl.h
    ${INCDIR}/PrintHelpers.h
    ${INCDIR}/LogBinding.h
)

set(MAIN_SOURCE_FILES 
    ${MOC_OUTFILES} 
    src/GraspItSceneManager.cpp 
    src/GraspItSceneManagerNoGui.cpp 
    src/GraspItAccessor.cpp 
    src/EigenGraspPlanner.cpp 
    src/EigenGraspPlannerNoQt.cpp 
    src/EigenGraspResult.cpp 
    src/LogBinding.cpp 
    src/PrintHelpers.cpp 
    src/GraspItHelpers.cpp
    src/GraspItDatabaseManager.cpp
    src/GraspItSimpleDBManager.cpp
)

## Declare a C++ library
add_library(grasp_planning_graspit SHARED
 ${MAIN_SOURCE_FILES} 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
if (COMPILE_WITH_CATKIN)
    add_dependencies(grasp_planning_graspit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (COMPILE_WITH_CATKIN)

if (BUILD_GRASPIT)
    add_dependencies(grasp_planning_graspit graspit_ext graspit_static)
endif (BUILD_GRASPIT)

## Declare C++ executables
add_executable(grasp_planning src/grasp_planning.cpp)
add_executable(simple_graspit_database test/simple_graspit_database.cpp)
add_executable(simple_planning_tutorial tutorial/src/simple_planning_tutorial.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
if (COMPILE_WITH_CATKIN)
    add_dependencies(grasp_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(simple_graspit_database ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(simple_planning_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (COMPILE_WITH_CATKIN)

# add depending libraries if they don't come with catkin dependencies already
if (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)
    set (DEPEND_LIBRARIES
       ${GRASPIT_SYSTEM_LIBS} 
       ${GRASPIT_LIBRARY}
    )
endif (NOT COMPILE_WITH_CATKIN OR BUILD_GRASPIT)

if (COMPILE_WITH_CATKIN)
    set (DEPEND_LIBRARIES ${catkin_LIBRARIES} ${DEPEND_LIBRARIES})
endif (COMPILE_WITH_CATKIN)

## Specify libraries to link a library or executable target against

target_link_libraries(grasp_planning_graspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

target_link_libraries(grasp_planning
  grasp_planning_graspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

target_link_libraries(simple_planning_tutorial
  grasp_planning_graspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

target_link_libraries(simple_graspit_database
  grasp_planning_graspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

# style checking
if (COMPILE_WITH_CATKIN)
    roslint_cpp()
endif (COMPILE_WITH_CATKIN)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS grasp_planning_graspit grasp_planning_graspit_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

if (COMPILE_WITH_CATKIN) 
    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
    set ( INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set ( LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set ( BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set ( SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else (COMPILE_WITH_CATKIN) 
    set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
    set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
    set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)
endif (COMPILE_WITH_CATKIN) 

## Mark executables and/or libraries for installation
install(TARGETS grasp_planning_graspit
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(TARGETS grasp_planning
   ARCHIVE DESTINATION ${BIN_DESTINATION}
   LIBRARY DESTINATION ${BIN_DESTINATION}
   RUNTIME DESTINATION ${BIN_DESTINATION}
)

if (BUILD_GRASPIT)
    if (COMPILE_WITH_CATKIN)
        # make the GraspIt! simulator available also in the catkin devel space
        add_custom_target(copy_simulator_exe ALL 
            COMMAND ${CMAKE_COMMAND} -E copy
                ${GRASPIT_EXECUTABLE} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${PROJECT_NAME}
            DEPENDS graspit_ext
        )
    endif (COMPILE_WITH_CATKIN)
    install(PROGRAMS ${GRASPIT_EXECUTABLE} 
       DESTINATION ${BIN_DESTINATION}
    )
endif (BUILD_GRASPIT)

install(FILES
    ${MAIN_HEADER_FILES}
    DESTINATION ${INCLUDE_DESTINATION}
)

install(DIRECTORY 
    CMakeMacros 
    DESTINATION ${SHARE_DESTINATION}
)

install(DIRECTORY 
    tutorial 
    DESTINATION ${SHARE_DESTINATION}
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)
    install(DIRECTORY 
        ${CMAKE_CURRENT_BINARY_DIR}/docs/html 
        DESTINATION ${SHARE_DESTINATION}/doc
    )
endif (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)

#############
## Testing ##
#############

if (COMPILE_WITH_CATKIN)
    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_planning_graspit.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
endif (COMPILE_WITH_CATKIN)
