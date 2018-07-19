# Which EigenGraspPlanner implementation to use?

There are two EigenGraspPlanner implementations:

1. EigenGraspPlanner: Uses the Qt signals sent from original GraspIt! EGPlanner class.
2. EigenGraspPlannerNoQt: Detaches from using Qt.


# 1. EigenGraspPLanner

The EigenGraspPlanner implementation controls the planner from within 
the SoQt thread.
This means that when plan() is called, only a status flag is set,
which is then read from within a callback called
from the SoQt thread (specifically, this callback is ivIdleCallback(),
which is called from idleEventFromSceneManager(),
or from a locally maintained SoIdleSensor).

From this callback, the planner is then started. 

## Advantages

- the QT signals and slots can be used and the planner can send the appropriate signals.
    Signals are sent from the GraspIt EGPlanner object to update the current state and to
    notify when the planning is finished. The slots EigenGraspPlanner::plannerUpdateSlot()
    and EigenGraspPlanner::plannerCompleteSlot() may connect to these signals.
- this methods it requires minimal changes to the original graspit code, due to the support
    of the signals. 

## Disadvantages

- dependencies to Qt are apparent in the implementation itself.
- the EigenGraspPlanner class needs to derive from QObject, and that the MOC files have to be generated with cmake.


# 2. EigenGraspPlannerNoQt

EigenGraspPlannerNoQt runs the planning algorithm from the calling
thread (the one which calls plan()). 
This requires a new method in the original graspit code (specifically, EGPlanner::runPlannerLoop()).

The method EGPlanner::runPlannerLoop() essentially does the same as EGPlanner::startThread() 
(only without starting a thread), and then calling run(). 
This is needed because all QObjects have to be created by
the same thread which also runs the planning. So the planning cannot be run as the
simulator is doing with EGPlanner (out of EGPlanner::sensorCB()).

UPDATE: This is currently not supported any more. The method ``EGPlanner::runPlannerLoop``
would need to be merged into the original graspit repository (my fork is outdated).

The code required:
```cpp
void 
EGPlanner::runPlannerLoop() 
{ 
    mMultiThread = false;

    PROF_RESET_ALL;
    PROF_START_TIMER(EG_PLANNER);

    mProfileInstance->startTimer();

    mRenderType = RENDER_NEVER;
    
    //signal that initialization is ready
    setState(RUNNING);
    threadLoop();
}

```

Additional comment about this method:    
This method essentially does the same as *startThread()* (without starting a thread) and then calling *run()*.
This is needed because all QObjects have to be created by the same thread which also runs the planning. So we cannot run the planning
as originally with *EGPlanner* (out of the method *EGPlanner::sensorCB*). 

## Advantages

- Does not require the EigenGraspPlanner class to use slots and derive from QObject. 
- Does not require updates from the GraspItSceneManager event loop.

## Disadvantages

- New method EGPlanner::runPlannerLoop() needs to be added to original graspit EGPlanner class.
    This represents a more invasive change to the original graspit source. 
- It is unfortunate hat the signals from EGPlanner cannot be received with this method.
