^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grasp_planning_graspit_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2018-01-06)
------------------

1.1.3 (2018-01-05)
------------------
* Adaptation for graspit upstream merge
* Changed link/finger names for new jaco
* Contributors: Jennifer Buehler

1.1.1 (2016-08-07)
------------------

1.1.0 (2016-07-26)
------------------

1.0.0 (2016-06-08)
------------------
* Now compiles with new graspit and new urdf2inventor
* Contributors: Jennifer Buehler

0.1.5 (2016-04-02)
------------------
* Added pre-grasp state computation
* Added helper class EigenGraspPlannerClient and some example grasp results for Jaco
* Now support addition of an auto-grasp as finalization of grasp planning, and control of how many results are saved
* Added means to save a Grasp.msg and added a table obstacle
* Contributors: Jennifer Buehler

0.1.3 (2016-03-08)
------------------
* added launch directory installs
* Adapted cmake file to for jenkins, and removed c++11 flag, which required fixing two little things in sources
* Contributors: Jennifer Buehler

0.1.2 (2016-03-04)
------------------

0.1.0 (2016-03-02)
------------------
* roslinted
* graspit fork is now compiled into libraries here and doesn't need to be installed separately
* Contributors: Jennifer Buehler
