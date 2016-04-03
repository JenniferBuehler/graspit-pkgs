- move grasp_planning_graspit(_ros) to new architecture_binding
- use the urdf2inventor dependency. FileIO and ConversionResult need to be derived though, as they contain GraspIt! specific things.
    MarkerSelector can probably use parts of InventorViewer. Before converting, should create a proper class hierarchy separated in general URDF traversal and operations
    such as mesh conversion and model scaling.
    When pulling in the dependency, can also get rid of compiling ivcon in urdf2graspit.
    ATTENTION: Contacts are not scaled any more in Urdf2Inventor::scaleTranslation()!! And convertMesh(RescursionParamsPtr) is not writing resultXMLDesc any more!!!
- add images to urdf2graspit tutorial


## older issues (maybe resolved?)

- Some weird problem still happens if running grasp_planning_node.launch with use_world:=false. For some reason it triggers loading
  QPixmap when loading the object. It only happens rarely. Could not figure out why this happens so far as it happens randomly.
- Very rarely there is still a segfault on exit of the program, caused by SoIdleSensor (I believe when running with USE_SEPARATE_SOSENSOR only?).
  Obviously through object destructors. Have to investigate why, this should not happen if listeners have unregistered themselves.

## undecided whether it is worth investigating

- GraspItAccessor: Could improve thead safety by providing a separate flag protected by its own mutex which indicates whether
  the world is locked for changes at the moment (e.g. a planner algorithm is running on it). At the moment the
  whole world mutex may get locked from GraspItAccessor classes which is fine to start with, but later we may want
  to handle this a bit looser.
- could make GraspItSceneManager adhere to a more general listener/event-publisher pattern, instead of only the "idle loop". IdleLoop could be one
 particular event that is supported by it.

# urdf2graspit

- add API generation to CMake
- check in XMLFuncs.cpp with getJointLimits() calls whether it has to be converted to degrees everywhere, though I think it is ok like this. I marked lines with XXX TODO.
- could also make this a stand-alone package independent of ROS, but then would have to make sure at least urdf libs are installed...
- would be nice to have an interface to do the Eigengrasps as well

