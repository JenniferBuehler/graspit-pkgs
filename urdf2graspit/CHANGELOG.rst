^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf2graspit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
