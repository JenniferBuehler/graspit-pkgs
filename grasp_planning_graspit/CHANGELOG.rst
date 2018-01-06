^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grasp_planning_graspit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2018-01-06)
------------------
* Update
* Contributors: Jennifer Buehler

1.1.3 (2018-01-05)
------------------
* Tested with current version of graspit
* Adaptation for graspit upstream merge
* Contributors: Jennifer Buehler

1.1.1 (2016-08-07)
------------------
* Contributors: Jennifer Buehler

1.1.0 (2016-07-26)
------------------
* New version to work with new update on graspit fork
* Removed camera parameters setting as this is done in GraspitCore now
* EGPlanner working with new graspit headless branch
* Updated command line parsing in grasp_planning.cpp and adjusted some parameters for prismatic joints in urdf2graspit
* removed more unneeded includes
* removed unneeded includes
* removed unneeded includes of ivmgrHeadless.h
* Added option to change object position and max number of iterations as command line argument in grasp_planning.cpp
* Now can save planning results without objects -only pose- as well
* Contributors: Jennifer Buehler

1.0.0 (2016-06-08)
------------------
* Removed direct Qt dependencies from urdf2graspit, as this is now in urdf2inventor
* Now compiles with new graspit and new urdf2inventor
* Contributors: Jennifer Buehler

0.1.5 (2016-04-02)
------------------
* Added pre-grasp state computation
* Added helper class EigenGraspPlannerClient and some example grasp results for Jaco
* Now supports an extra auto-grasp as finalization of planning, and control of how many results are saved
* Fixed broken hand transform which was returned for resulting grasps
* Added means to save a Grasp.msg and added a table obstacle
* Contributors: Jennifer Buehler

0.1.3 (2016-03-08)
------------------
* changed catkin_package system depends to names in rosdep
* Adapted cmake file to for jenkins, and removed c++11 flag, which required fixing two little things in sources
* Contributors: Jennifer Buehler

0.1.2 (2016-03-04)
------------------

0.1.0 (2016-03-02)
------------------
* now also adds graspit_simulator_standalone
* graspit fork is now compiled into libraries here and doesn't need to be installed separately
* Contributors: Jennifer Buehler
