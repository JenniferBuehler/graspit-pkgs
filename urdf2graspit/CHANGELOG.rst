^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf2graspit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2018-01-05)
------------------
* Adaptation for graspit upstream merge
* Fixed bug for issue `#19 <https://github.com/JenniferBuehler/graspit-pkgs/issues/19>`_
* Fixed problem with contact points in which names not accepted by SoName would then not be found in the MarkerSelector
* Fixed bug: transform to DH world frame in Urdf2Graspit.cpp
* Added urdf viewer launch file for jaco
* Rotation axis correction was the wrong way, now negate_joint_movement may not be necessary for some hands (or become necessary for others
* More testing changes
* Backup
* Fixed issue with thumb in allegro but still not displaying properly in GraspIt simulator
* Improved debugging abilities with viewer
* Fixed missing white spaces in XMLFuncs.cpp generation of inertia
* Catching case in urdf2graspit for hands with no inertial. Fixes `#19 <https://github.com/JenniferBuehler/graspit-pkgs/issues/19>`_
* Changed link/finger names for new jaco
* Contributors: Jennifer Buehler

1.1.1 (2016-08-07)
------------------
* Fix cylinder and box orientations
* Contact generation bug fixed
* Corrected urdf2graspit contact generation to place cylinder at right spot in Baxter
* Contributors: Jennifer Buehler

1.1.0 (2016-07-26)
------------------
* minor fix in generating eigen.xml
* Indented xml
* Jenkins fix
* Updated command line parsing in grasp_planning.cpp and adjusted some parameters for prismatic joints in urdf2graspit
* Fix minor print bug for fixing joints
* Contributors: Jennifer Buehler

1.0.0 (2016-06-08)
------------------
* Removed direct Qt dependencies from urdf2graspit, as this is now in urdf2inventor
* Now working with new urdf2inventor, tested on jaco
* Fixed some bugs in DH parameter computation
* Can now be used to display local coordinate axes of DH parameters, useful for debugging
* Allows to save the contacts file as different filename
* Contact selection is now done in separate postprocessing step
* Contacts can now be generated in a post-processing step.
* Contacts can now be generated in a post-processing step. This commit includes the old code within ifndef 0 directives
* urdf2graspit now depends on urdf2inventor
* Contributors: Jennifer Buehler

0.1.5 (2016-04-02)
------------------
* Small fix: create mesh directory if the mesh filename implies more directories
* Contributors: Jennifer Buehler

0.1.3 (2016-03-08)
------------------
* changed catkin_package system depends to names in rosdep
* added launch directory installs
* Contributors: Jennifer Buehler

0.1.2 (2016-03-04)
------------------

0.1.0 (2016-03-02)
------------------
* made ivcon external
* roslinted
* Contributors: Jennifer Buehler
