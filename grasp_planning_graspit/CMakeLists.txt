cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_graspit)

set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem system thread program_options)

# catkin is optional, and only to be used when added 
# as ROS package and compiling with catkin_make. 
# CATKIN_DEVEL_PREFIX is only defined if building with catkin
# TODO: Would be nicer to use another variable here like
# BUILDING_WITH_CATKIN if something like that exists?
if (CATKIN_DEVEL_PREFIX)
    message("Building grasp_plannin_graspit with catkin.")
    find_package(catkin REQUIRED COMPONENTS graspit eigen_conversions) 
else (CATKIN_DEVEL_PREFIX)
    message("Building grasp_plannin_graspit without catkin.")
endif (CATKIN_DEVEL_PREFIX) 

# if catkin is not used, we have to get the Qt dependencies separately
if (NOT catkin_FOUND)
    find_package (graspit REQUIRED)
    find_package(Qhull REQUIRED)
    find_package(SoQt4 REQUIRED)
    find_package(LAPACK REQUIRED)
    find_package(Threads REQUIRED)

    SET( QT_USE_QT3SUPPORT TRUE )
    find_package(Qt4 COMPONENTS QtCore REQUIRED)

    # include QT_USE_FILE is needed.
    include (${QT_USE_FILE})
endif (NOT catkin_FOUND)


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



# include QT_USE_FILE is not necessary, because it is included via the catkin dependencies.
# Qt MOC files are needed ONLY because QT slots are used at the moment in EigenGraspPlanner.h
# Qt include files should be covered by the implicit dependency of graspit libraries
# if compiled with catkin
set (MOC_HEADERS include/grasp_planning_graspit/EigenGraspPlanner.h) 
QT4_WRAP_CPP(MOC_OUTFILES ${MOC_HEADERS})
     
if (catkin_FOUND)
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
      LIBRARIES graspPlanningGraspit
      CATKIN_DEPENDS graspit eigen_conversions 
      DEPENDS ${Boost_LIBRARIES}
    )
endif (catkin_FOUND)


###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11 
    -g
    -DGRASPITDBG 
)

# message ("++++++++++++++++++++++++++++++++++++++++")
# message (${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
else (catkin_FOUND)
    include_directories(
         ${GRASPIT_INCLUDE_DIRS}
         ${SOQT_INCLUDE_DIRS}
         ${QT_INCLUDES}
         ${QT_INCLUDE_DIR}
         ${QT_QT3SUPPORT_INCLUDE_DIR}
         ${QHULL_INCLUDE_DIRS}
    )
endif (catkin_FOUND)

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
    ${INCDIR}/GraspItHelpers.h
    ${INCDIR}/SharedPtr.h
    ${INCDIR}/ThreadImpl.h
    ${INCDIR}/PrintHelpers.h
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
add_library(graspPlanningGraspit SHARED
 ${MAIN_SOURCE_FILES} 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
if (catkin_FOUND)
    add_dependencies(graspPlanningGraspit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (catkin_FOUND)

## Declare C++ executables
add_executable(grasp_planning src/grasp_planning.cpp)
#add_executable(simple_planning_tutorial tutorial/src/simple_planning_tutorial.cpp)
add_executable(simple_graspit_database test/simple_graspit_database.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
if (catkin_FOUND)
    add_dependencies(grasp_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#    add_dependencies(simple_planning_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(simple_graspit_database ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (catkin_FOUND)

# add depending libraries if they don't come with catkin dependencies already
if (NOT catkin_FOUND)
    set (DEPEND_LIBRARIES
       ${QT_LIBRARIES}
       ${QT_QT3SUPPORT_LIBRARY}
       ${QHULL_LIBRARIES}
       ${SOQT_LIBRARY}
       ${LAPACK_LIBRARIES}
       ${GRASPIT_LIBRARY}
    )
else (NOT catkin_FOUND)
    set (DEPEND_LIBRARIES ${catkin_LIBRARIES})
endif (NOT catkin_FOUND)

## Specify libraries to link a library or executable target against

target_link_libraries(graspPlanningGraspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

target_link_libraries(grasp_planning
  graspPlanningGraspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)


#target_link_libraries(simple_planning_tutorial
#  graspPlanningGraspit
#  ${Boost_LIBRARIES}
#  ${DEPEND_LIBRARIES}
#)

target_link_libraries(simple_graspit_database
  graspPlanningGraspit
  ${Boost_LIBRARIES}
  ${DEPEND_LIBRARIES}
)

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



if (catkin_FOUND) 
    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
    set ( INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set ( LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set ( BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    set ( SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else (catkin_FOUND) 
    set ( INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
    set ( LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    set ( BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
    set ( SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)
endif (catkin_FOUND) 



## Mark executables and/or libraries for installation
install(TARGETS grasp_planning
   ARCHIVE DESTINATION ${LIB_DESTINATION}
   LIBRARY DESTINATION ${LIB_DESTINATION}
   RUNTIME DESTINATION ${LIB_DESTINATION}
)

install(TARGETS grasp_planning
   ARCHIVE DESTINATION ${BIN_DESTINATION}
   LIBRARY DESTINATION ${BIN_DESTINATION}
   RUNTIME DESTINATION ${BIN_DESTINATION}
)

install(FILES
    ${MAIN_HEADER_FILES}
    DESTINATION ${INCLUDE_DESTINATION}
)

install(DIRECTORY 
    CMakeMacros 
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

if (catkin_FOUND)
    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_planning_graspit.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
endif (catkin_FOUND)
