/**
   Implementation of the GraspIt planner interface.

   Copyright (C) 2016 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/GraspItHelpers.h>

#include <string>
#include <vector>
#include <algorithm>

#include <graspit/matvec3D.h>
#include <graspit/world.h>
#include <graspit/robot.h>
#include <graspit/body.h>
#include <graspit/grasp.h>
#include <graspit/eigenGrasp.h>

#include <graspit/EGPlanner/search.h>
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/energy/searchEnergy.h>
#include <graspit/EGPlanner/egPlanner.h>
#include <graspit/EGPlanner/simAnn.h>
#include <graspit/EGPlanner/simAnnPlanner.h>

#include <QWidget>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/sensors/SoIdleSensor.h>

#include <boost/filesystem.hpp>

#define AUTOGRASP_VELOCITY 0.05
#define QUICKOPEN_FACTOR 0.01

using GraspIt::EigenGraspPlanner;
using GraspIt::Log;

EigenGraspPlanner::EigenGraspPlanner(const std::string& name, const SHARED_PTR<GraspItSceneManager>& intr):
    GraspItAccessor(name, intr),
    // mEnergyCalculator(NULL),
    graspitEgPlanner(NULL),
    graspitStateType(AxisAngle),
    graspitSearchEnergyType(EnergyContact),
    useContacts(true),
#ifdef USE_SEPARATE_SOSENSOR
    mIdleSensor(NULL),
#endif
    planCommand(NONE)
{
    // mEnergyCalculator=new SearchEnergy();
    // mEnergyCalculator->setStatStream(&std::cout);

    if (!eventThreadRunsQt())
    {
        PRINTERROR("EigenGraspPlanner supports only GraspItSceneManager instances which run Qt.");
        throw std::string("EigenGraspPlanner supports only GraspItSceneManager instances which run Qt.");
    }

    addAsIdleListener();
}

EigenGraspPlanner::~EigenGraspPlanner()
{
    PRINTMSG("EigenGrasp planner destructor");

    removeFromIdleListeners();

#ifdef USE_SEPARATE_SOSENSOR
    // quit the idle sensor to avoid conflicts when Inventor
    // is shut down
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled())
        {
            mIdleSensor->unschedule();
        }
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
#endif

    graspitEgPlannerMtx.lock();
    if (graspitEgPlanner)
    {
        delete graspitEgPlanner;
        graspitEgPlanner = NULL;
    }
    graspitEgPlannerMtx.unlock();

    deleteResults();

    // if (mEnergyCalculator) delete mEnergyCalculator;
    PRINTMSG("Exit EigenGrasp planner destructor");
}


void EigenGraspPlanner::onSceneManagerShutdown()
{
#ifdef USE_SEPARATE_SOSENSOR
    // quit the idle sensor to avoid conflicts when Inventor
    // is shut down
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled())
        {
            mIdleSensor->unschedule();
        }
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
#endif

    graspitEgPlannerMtx.lock();
    if (graspitEgPlanner)
    {
        delete graspitEgPlanner;
        graspitEgPlanner = NULL;
    }
    graspitEgPlannerMtx.unlock();
}

void EigenGraspPlanner::idleEventFromSceneManager()
{
#ifdef USE_SEPARATE_SOSENSOR
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
    }

    // start the idle sensor which is triggered regularly from within the inventor thread
    mIdleSensor = new SoIdleSensor(sensorCB, this);
    mIdleSensor->schedule();
#else
    scheduleForIdleEventUpdate();
    ivIdleCallback();
#endif
}




void EigenGraspPlanner::plannerUpdateSlot()
{
    plannerUpdate();
}

void EigenGraspPlanner::plannerCompleteSlot()
{
    PRINTMSG("Planning complete!!");
    setPlannerCommand(STOP);
}


EigenGraspPlanner::PlannerCommand EigenGraspPlanner::getPlannerCommand()
{
    UNIQUE_LOCK lck(planCommandMtx);
    return planCommand;
}

void EigenGraspPlanner::setPlannerCommand(const PlannerCommand c)
{
    UNIQUE_LOCK lck(planCommandMtx);
    planCommand = c;
}


#ifdef USE_SEPARATE_SOSENSOR
void EigenGraspPlanner::sensorCB(void *data, SoSensor *)
{
    // PRINTMSG(" ### sensorCB ###");
    EigenGraspPlanner* _this = dynamic_cast<EigenGraspPlanner*>(static_cast<EigenGraspPlanner*>(data));
    if (!_this)
    {
        PRINTERROR("Could not cast EigenGraspPlanner");
        return;
    }

    _this->ivIdleCallback();

    // trigger another call of this method in the next event loop iteration
    _this->mIdleSensor->schedule();
}
#endif

void EigenGraspPlanner::ivIdleCallback()
{
    if (!getGraspItSceneManager()->isInitialized())
    {
        PRINTWARN("Graspit scene manager not initialized.");
        return;
    }

    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (getPlannerCommand() == START)
    {
        PRINTMSG("### Start planner! ###");
        assert(graspitEgPlanner);
        if (!graspitEgPlanner)
        {
            PRINTERROR("EigenGraspPlanner not initialized");
        }
        else
        {
            if (!QObject::connect(graspitEgPlanner, SIGNAL(update()),
                                  this, SLOT(plannerUpdateSlot()), Qt::DirectConnection) ||
                    !QObject::connect(graspitEgPlanner, SIGNAL(complete()),
                                      this, SLOT(plannerCompleteSlot()), Qt::DirectConnection))
            {
                PRINTERROR("Could not connect signals and slots");
            }
            else
            {
                PRINTMSG("Entering planner loop...");
                // start the planner. This method is non-blocking:
                // it kicks off the planner in a separate thread.
                graspitEgPlanner->startPlanner();
            }
            // remove the starting flag, because the planner has either started
            // or a fatal error occurred.
            setPlannerCommand(NONE);
        }
    }
    else if (getPlannerCommand() == STOP)
    {
        if (!QObject::connect(graspitEgPlanner, SIGNAL(update()),
                              this, SLOT(plannerUpdateSlot()), Qt::DirectConnection) ||
                !QObject::connect(graspitEgPlanner, SIGNAL(complete()),
                                  this, SLOT(plannerCompleteSlot()), Qt::DirectConnection))
        {
            PRINTERROR("Could not disconnect signals and slots");
        }
    }
}


bool EigenGraspPlanner::plan(const std::string& handName, const std::string& objectName,
                             const EigenTransform * objectPose,
                             const int maxPlanningSteps,
                             const int repeatPlanning,
                             const int maxResultsPerRepeat,
                             const bool finishWithAutograsp,
                             const PlannerType& planType)
{
    if (!getGraspItSceneManager()->isInitialized())
    {
        PRINTERROR("Graspit scene manager not initialized. Cannot do planning.");
        return false;
    }

    // lock the world so that it won't change while we set the hand/object and do the planning
    UNIQUE_RECURSIVE_LOCK lock = getUniqueWorldLock();
    if (getGraspItSceneManager()->setCurrentHand(handName) != 0)
    {
        PRINTERROR("Could not set current hand " << handName);
        return false;
    }
    if (getGraspItSceneManager()->setCurrentGraspableObject(objectName) != 0)
    {
        PRINTERROR("Could not set current object " << objectName);
        return false;
    }
    if (objectPose && (getGraspItSceneManager()->moveObject(objectName, *objectPose) != 0))
    {
        PRINTERROR("Could not set the object pose");
        return false;
    }

    // for testing we can save the start world to see how it looks before we start planning
    // getGraspItSceneManager()->saveGraspItWorld("/home/jenny/test/worlds/startWorld.xml",true);
    // getGraspItSceneManager()->saveInventorWorld("/home/jenny/test/worlds/startWorld.iv",true);

    return plan(maxPlanningSteps, repeatPlanning, maxResultsPerRepeat, finishWithAutograsp, planType);
}

bool compareGraspPlanningStates(const GraspPlanningState* g1, const GraspPlanningState* g2)
{
    return g1->getEnergy() < g2->getEnergy();
}

void EigenGraspPlanner::deleteResults()
{
    for (std::vector<const GraspPlanningState*>::iterator it=results.begin(); it!=results.end(); ++it)
    {
        delete *it;
    }
    results.clear();
}

bool EigenGraspPlanner::plan(const int maxPlanningSteps,
                             const int repeatPlanning,
                             const int maxResultsPerRepeat,
                             const bool finishWithAutograsp,
                             const PlannerType& planType)
{
    if (!getGraspItSceneManager()->isInitialized())
    {
        PRINTERROR("Graspit scene manager not initialized. Cannot do planning.");
        return false;
    }

    deleteResults();

    // lock the world so that it won't change while we do the planning
    UNIQUE_RECURSIVE_LOCK lock = getUniqueWorldLock();

    for (int i = 0; i < repeatPlanning; ++i)
    {
        //PRINTMSG("Initializing planning...");
        if (!initPlanner(maxPlanningSteps, planType))
        {
            PRINTERROR("Could not initialize planner.");
            return false;
        }
        {
            UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
            if (!graspitEgPlanner)
            {
                PRINTERROR("EigenGraspPlanner not initialized");
                return false;
            }

            if (graspitEgPlanner->isActive())
            {
                PRINTERROR("EigenGraspPlanner is already active, you cannot call plan()");
                return false;
            }

            if (!graspitEgPlanner->isReady())
            {
                PRINTERROR("EigenGraspPlanner is not ready.");
                return false;
            }
        }

        PRINTMSG("Initiating planning, trial #"<<i);

        scheduleForIdleEventUpdate();
        setPlannerCommand(START);

        PRINTMSG("Waiting for planning algorithm to finish...");
        // this is the wait loop, ONLY needed for option a)
        while (getPlannerCommand() != STOP)
        {
            // PRINTMSG("EigenGraspPlanner is running");
            SLEEP(0.1);
        }
        // re-set planner command to none because now the stopping has been registered in this thread.
        setPlannerCommand(NONE);

        plannerComplete();
        PRINTMSG("Planning done.");

        // store results
        graspitEgPlannerMtx.lock();

        int numGrasps = std::min(graspitEgPlanner->getListSize(), maxResultsPerRepeat);
        for (int i = 0; i < numGrasps; ++i)
        {
            const GraspPlanningState *sOrig = graspitEgPlanner->getGrasp(i);
            GraspPlanningState * sCopy = new GraspPlanningState(sOrig);
            if (finishWithAutograsp)
            {
                if (!sCopy->execute())
                {
                    PRINTERROR("Could not execute grasp state for grasp "<<i<<", skipping auto-grasp");
                    continue;
                }
                else
                {
                    if (!sCopy->getHand()->autoGrasp(false,AUTOGRASP_VELOCITY,false))
                    {
                        PRINTWARN("Could not perform autograsp for grasp "<<i);
                    }
                    else
                    {
                         sCopy->saveCurrentHandState();
                    }
                }
            }
            results.push_back(sCopy);
        }
        graspitEgPlannerMtx.unlock();
    }
    std::sort(results.begin(), results.end(), compareGraspPlanningStates);
    return true;
}

/*void EigenGraspPlanner::stopPlanner()
{
    graspitEgPlannerMtx.lock();
    graspitEgPlanner->pausePlanner();
    graspitEgPlannerMtx.unlock();
}*/

GraspIt::EigenTransform EigenGraspPlanner::getHandTransform(const GraspPlanningState * s) const
{
    EigenTransform handTransform = getEigenTransform(s->getTotalTran());
    return handTransform;
}

GraspIt::EigenTransform EigenGraspPlanner::getObjectTransform(const GraspPlanningState * s) const
{
    GraspableBody * object = s->getObject();
    if (!object)
    {
        PRINTERROR("Object not initialized");
        return EigenTransform::Identity();
    }

    transf pose = object->getTran();
    Quaternion q = pose.rotation();
    vec3 t = pose.translation();

    // PRINTMSG("Object position: "<<t.x()<<"/"<<t.y()<<"/"<<t.z());
    // PRINTMSG("Object quaternion: "<<q.x<<"/"<<q.y<<"/"<<q.z<<"/"<<q.w);

    EigenTransform objectTransform;
    objectTransform.setIdentity();
    Eigen::Vector3d eObjectTranslation(t.x(), t.y(), t.z());
    Eigen::Quaterniond eObjectQuaternion(q.w(), q.x(), q.y(), q.z());

    objectTransform = objectTransform.translate(eObjectTranslation);
    objectTransform = objectTransform.rotate(eObjectQuaternion);

    return objectTransform;
}

void EigenGraspPlanner::getGraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const
{
    if (!s->getHand())
    {
        PRINTERROR("Hand is NULL!");
        return;
    }
    const PostureState* handPosture = s->readPosture();
    if (!handPosture)
    {
        PRINTERROR("Posture is NULL!");
        return;
    }

    // this gets the DOF values for the 3 fingers. If the hand has only one EigenGrasp which is symmetric,
    // the 3 values will be the same.
    const int numDOF = s->getHand()->getNumDOF();
    double * _dofs = new double[numDOF];
    handPosture->getHandDOF(_dofs);
    // PRINTMSG("Grasp DOFs: ");
    for (int k = 0; k < numDOF; ++k)
    {
        // PRINTMSG(_dofs[k]);
        dofs.push_back(_dofs[k]);
    }
}


void EigenGraspPlanner::getPregraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const
{
    GraspPlanningState sCopy(s);
    if (!sCopy.getHand())
    {
        PRINTERROR("Hand is NULL!");
        return;
    }
    const int numDOF = sCopy.getHand()->getNumDOF();
    const PostureState* handPosture = sCopy.readPosture();
    if (!handPosture)
    {
        PRINTERROR("Posture is NULL!");
        return;
    }
    
    // execute the auto-grasp in "opening" direction
    if (!sCopy.execute())
    {
        PRINTWARN("Could not execute grasp state, the pre-grasp state may not be ideal.");
    }
    // opens hand until no collision is detected
    if (!sCopy.getHand()->quickOpen(QUICKOPEN_FACTOR))
    {
        PRINTMSG("INFO - EigenGraspPlanner: quickOpen() returned false. The hand may have been completely opened (quickOpen() is a Graspit! hack with no alternative so far, as auto-grasp requires collision-free state)");
    }

    // XXX test print:
    /*double * __dofs = new double[numDOF];
    sCopy.getHand()->getDOFVals(__dofs);
    PRINTMSG("After quickopen DOFs: ");
    for (int k = 0; k < numDOF; ++k)
    {
        PRINTMSG(__dofs[k]);
    }*/
    // XXX end test print

    // re-close grip until first contact, this should also retreat a tiny bit,
    // so that there are no more contacts
    /*if (!sCopy.getHand()->autoGrasp(false,AUTOGRASP_VELOCITY,true))
    {
        PRINTWARN("Could not correctly re-close hand with auto-grasp. The resulting pre-grasp state may not be ideal.");
    }*/

    // opens hand until a collision is detected. This will open the hand either all the way,
    // or not all the way if collision detected.
    // Prerequisite: Hand may not be in collision, otherwise this gets interpolation errors and 
    // returns false
    if (!sCopy.getHand()->autoGrasp(false,-AUTOGRASP_VELOCITY,true))
    {
        PRINTWARN("Could not correctly open hand with auto-grasp, the pre-grasp state may not be ideal.");
    }
    // in case a collision stopped the opening of hand, close the grip again
    // until no collision is detected. 
    /*if (!sCopy.getHand()->quickOpen(-QUICKOPEN_FACTOR))
    {
        PRINTWARN("Could not correctly open hand with auto-grasp, the pre-grasp state may not be ideal.");
    }*/
    sCopy.saveCurrentHandState();

    // this gets the DOF values for the 3 fingers. If the hand has only one EigenGrasp which is symmetric,
    // the 3 values will be the same.
    double * _dofs = new double[numDOF];
    handPosture->getHandDOF(_dofs);
    //PRINTMSG("Pre-grasp DOFs: ");
    for (int k = 0; k < numDOF; ++k)
    {
        //PRINTMSG(_dofs[k]);
        dofs.push_back(_dofs[k]);
    }
}



void EigenGraspPlanner::getEigenGraspValues(const GraspPlanningState * s, std::vector<double>& egVals) const
{
    if (!s->readPosture())
    {
        PRINTERROR("Posture is NULL!");
        return;
    }

    const PostureState* handPosture = s->readPosture();

    // print the EigenGrasp value
    for (int k = 0; k < handPosture->getNumVariables(); ++k)
    {
        // PRINTMSG("EigenGrasp val " << k << ": " << handPosture->getVariable(k)->getValue());
        egVals.push_back(handPosture->getVariable(k)->getValue());
    }
}

bool EigenGraspPlanner::checkStateValidity(const GraspPlanningState * s) const
{
    if (!s->readPosture() || !s->readPosition())
    {
        PRINTERROR("Posture or Position is NULL!");
        return false;
    }

    const PostureState* handPosture = s->readPosture();
    const PositionState* handPosition = s->readPosition();

    StateType postureType = handPosture->getType();
    if (postureType != POSE_EIGEN)
    {
        // TODO: testing neeed with other state types before we allow this
        PRINTERROR("Check if implementation for other pose types than Eigen work!");
        return false;
    }
    StateType positionType = handPosition->getType();
    if (positionType != SPACE_AXIS_ANGLE)
    {
        // TODO: testing neeed with other state types before we allow this
        PRINTERROR("Check if implementation for other position types than Axis-Angle work!");
        return false;
    }

    // we assume that the hand has not changed since we started the plannning
    const Hand * mHand = readCurrentHand();
    if (!mHand)
    {
        PRINTERROR("Hand is NULL!");
        return false;
    }
    if (mHand != s->getHand())
    {
        PRINTERROR("We have changed hand pointer!!!");
        return false;
    }

    return true;
}


bool EigenGraspPlanner::saveResultsAsWorldFiles(const std::string& inDirectory,
        const std::string& fileNamePrefix, bool asGraspIt,
        bool asInventor, bool createDir,
        bool saveSeparatePoseIV)
{
    if (!asGraspIt && !asInventor)
    {
        PRINTERROR("Have to specify either to save results as GraspIt or as Inventor files");
        return false;
    }

    if (!getGraspItSceneManager()->isInitialized())
    {
        PRINTERROR("Graspit scene manager not initialized.");
        return false;
    }

    // retrieve robot name if separate pose file should be saved
    std::string robotName;
    if (saveSeparatePoseIV)
    {
        std::vector<std::string> robs = getGraspItSceneManager()->getRobotNames();       
        if (robs.empty())
        {
            PRINTERROR("No robots loaded");
            return false;
        }
        if (robs.size()!=1)
        {
            PRINTERROR("Exactly 1 robot should have been loaded");
            return false;
        }
        robotName=robs.front();
    }

    int numGrasps = results.size();
    for (int i = 0; i < numGrasps; ++i)
    {
        const GraspPlanningState * s = results[i];
        if (!s)
        {
            PRINTERROR("GraspPlanningState is NULL!");
            return false;
        }

        if (!checkStateValidity(s))
        {
            PRINTERROR("Cannot work with the state of grasp " << i);
            return false;
        }

        // Execute grasp so that the correct world is saved
        if (!s->execute())
        {
            PRINTERROR("Could not execute grasp state for grasp "<<i<<", won't save the result");
            continue;
        }

        std::stringstream _wFilename;
        _wFilename << inDirectory << "/" << fileNamePrefix << "_" << (i + 1);
        std::string wFilename = _wFilename.str();
        if (asGraspIt && !getGraspItSceneManager()->saveGraspItWorld(wFilename + ".xml", createDir))
        {
            PRINTERROR("GraspIt could not save world file " << i);
            return false;
        }
        if (asInventor && !getGraspItSceneManager()->saveInventorWorld(wFilename + ".iv", createDir))
        {
            PRINTERROR("GraspIt could not save world file " << i);
            return false;
        }
        bool forceWrite = createDir;  // only enforce if creating dir is also allowed
        if (saveSeparatePoseIV && !getGraspItSceneManager()->saveRobotAsInventor(wFilename + "_pose.iv", robotName, createDir, forceWrite))
        {
            PRINTERROR("GraspIt could not save robot pose file " << i);
        }
    }
    return true;
}



bool EigenGraspPlanner::copyResult(const GraspPlanningState * s, EigenGraspResult& result) const
{
    if (!checkStateValidity(s))
    {
        PRINTERROR("Cannot work with this state");
        return false;
    }

    EigenTransform handTransform = getHandTransform(s);
    //PRINTMSG("RESULT "<<i<<" Hand transform: " << handTransform);

    EigenTransform objectTransform = getObjectTransform(s);
    //PRINTMSG("RESULT "<<i<<" Object transform: " << objectTransform);

    // Compute hand transform relative to object
    // objectTransform * relTransform = handTransform
    // relTransform = objectTransform.inverse() * handTransform;
    EigenTransform relTransform = objectTransform.inverse() * handTransform;
    
    //PRINTMSG("RESULT "<<i<<" rel transform: " << relTransform);

    // PRINTMSG("Relative transform: " << relTransform);

    // PRINTMSG("-- getting (pre)grasp DOFs --");
    std::vector<double> graspDOFs;
    getGraspJointDOFs(s, graspDOFs);
    /*     int k=0;
         for (std::vector<double>::const_iterator it=graspDOFs.begin(); it!=graspDOFs.end(); ++it){
             PRINTMSG("Grasp hand DOF "<<k<<": "<<*it);
             ++k;
         }
    */
    std::vector<double> pregraspDOFs;
    getPregraspJointDOFs(s, pregraspDOFs);
    /*     int k=0;
         for (std::vector<double>::const_iterator it=pregraspDOFs.begin(); it!=pregraspDOFs.end(); ++it){
             PRINTMSG("Grasp hand DOF "<<k<<": "<<*it);
             ++k;
         }
    */
    std::vector<double> egVals;
    getEigenGraspValues(s, egVals);
    /*      for (std::vector<double>::const_iterator it=egVals.begin(); it!=egVals.end(); ++it){
              PRINTMSG("EigenGrasp value "<<k<<": "<<*it);
              ++k;
          }*/
    result = EigenGraspResult(relTransform, graspDOFs, pregraspDOFs, egVals,
                s->isLegal(), s->getEpsilonQuality(), s->getVolume(), s->getEnergy());
    return true;
}




void EigenGraspPlanner::getResults(std::vector<EigenGraspResult>& allGrasps) const
{
    int numGrasps = results.size();
    for (int i = 0; i < numGrasps; ++i)
    {
        const GraspPlanningState * s = results[i];
        EigenGraspResult res;
        if (!copyResult(s,res))
        {
            PRINTERROR("Cannot work with this state");
            continue;
        }
        allGrasps.push_back(res);
    }
}


std::string getSearchEnergyType(const EigenGraspPlanner::GraspItSearchEnergyType& st)
{
  std::string ret;
    switch (st)
    {
    case EigenGraspPlanner::EnergyContact:
    {
        ret = "CONTACT_ENERGY";
        break;
    }
    // CONTACT_ENERGY, POTENTIAL_QUALITY_ENERGY, CONTACT_QUALITY_ENERGY,
    // AUTOGRASP_QUALITY_ENERGY, GUIDED_AUTOGRASP_ENERGY, STRICT_AUTOGRASP_ENERGY,
    // COMPLIANT_ENERGY, DYNAMIC_ENERGY
    default:
    {
        PRINTERROR("Unsupported search type");
    }
    }
    return ret;
}




StateType getStateType(const EigenGraspPlanner::GraspItStateType& st)
{
    StateType ret;
    switch (st)
    {
    case EigenGraspPlanner::AxisAngle:
    {
        ret = SPACE_AXIS_ANGLE;
        break;
    }
    /*    case SPACE_AXIS_ANGLE:
        case SPACE_COMPLETE:
        case SPACE_ELLIPSOID:
        case SPACE_APPROACH:
    */
    default:
    {
        PRINTERROR("Unsupported search type");
    }
    }
    return ret;
}





bool EigenGraspPlanner::initPlanner(const int maxPlanningSteps, const PlannerType& plannerType)
{
    Hand * mHand = getCurrentHand();
    GraspableBody * mObject = getCurrentGraspableBody();
    if (!mHand || !mObject)
    {
        PRINTERROR("Cannot initialize planner if no current hand and/or object is loaded");
        return false;
    }

    if (!mHand->getEigenGrasps())
    {
        PRINTERROR("Current hand has no EigenGrasp information!");
        return false;
    }

    if (!mHand->getGrasp())
    {
        PRINTERROR("Grasp is NULL!");
        return false;
    }

    if (mHand->getGrasp()->getObject() != mObject)
    {
        // this should already be set, but if it wasn't we could do it with this:
        // mHand->getGrasp()->setObjectNoUpdate(mObject);
        PRINTERROR("Consistency: Currently loaded object to grasp should be the same!");
        return false;
    }

    // mHand->getGrasp()->setGravity(true);
    mHand->getGrasp()->setGravity(false);
    GraspPlanningState graspPlanningState(mHand);
    graspPlanningState.setObject(mObject);
    StateType _stateType = getStateType(graspitStateType);
    graspPlanningState.setPositionType(_stateType);
    graspPlanningState.setRefTran(mObject->getTran());
    graspPlanningState.reset();

    /*
    // initialize variable types for this search:
    for (int i=0; i<graspPlanningState.getNumVariables(); i++) {
        // if checkbox in interface is checked:
        graspPlanningState.getVariable(i)->setFixed(false);
        graspPlanningState.getVariable(i)->setConfidence(0.0);
    }*/

    graspitEgPlannerMtx.lock();

    graspitEgPlanner = NULL;

    initSearchType(graspPlanningState, graspitStateType);

    initPlannerType(graspPlanningState, plannerType);

    setPlanningParameters();
    // steps
    graspitEgPlanner->setMaxSteps(maxPlanningSteps);

    graspitEgPlannerMtx.unlock();

    // print stats of the current grasp planning state
    /*    for (int i=0; i<graspPlanningState.getNumVariables(); i++) {
            std::string varName=graspPlanningState.getVariable(i)->getName().toStdString();
            float varValue=graspPlanningState.getVariable(i)->getValue();
            bool varFixed=graspPlanningState.getVariable(i)->isFixed();
            float varConf=graspPlanningState.getVariable(i)->getConfidence();
            PRINTMSG("Variable: "<<varName<<", value="<<varValue<<", fixed="<<varFixed<<", conf="<<varConf);
        }
    */
    return true;
}



void EigenGraspPlanner::initSearchType(GraspPlanningState& graspPlanningState, const GraspItStateType& st)
{
    StateType _stateType = getStateType(st);
    graspPlanningState.setPositionType(_stateType);

    GraspableBody * mObject = getCurrentGraspableBody();
    if (!mObject)
    {
        PRINTERROR("Object is NULL!");
        return;
    }

    switch (st)
    {
        /*    case SPACE_AXIS_ANGLE:
            case SPACE_COMPLETE:
            case SPACE_ELLIPSOID:*/
    case AxisAngle:
    {
        graspPlanningState.setRefTran(mObject->getTran());
        break;
    }
    /*    case SPACE_APPROACH:
        {
            Hand * mHand = getCurrentHand();
            if (!mHand)
            {
                PRINTERROR("Hand is NULL!");
                return;
            }
            graspPlanningState.setRefTran(mHand->getTran());
            break;
        }*/
    default:
    {
        PRINTERROR("Unsupported search type");
    }
    }

    graspPlanningState.reset();

    // force a reset of the planner
    graspitEgPlannerMtx.lock();
    if (graspitEgPlanner)
    {
        graspitEgPlanner->invalidateReset();
    }  // if graspitEgPlanner is NULL, it will have to be created again in initPlannerType()
    graspitEgPlannerMtx.unlock();
}



void EigenGraspPlanner::initPlannerType(const GraspPlanningState& graspPlanningState, const PlannerType &pt)
{
    PRINTMSG("Initializing planner type");


    Hand * mHand = getCurrentHand();
    if (!mHand)
    {
        PRINTERROR("Hand is NULL!");
        return;
    }

    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);

    // tell the planner which GraspPlanningState parameters to use by the graspPlanningState object.
    // the graspPlanningState object is only used as template in the functions called below (the object
    // is copied by the graspitEgPlanner object)
    switch (pt)
    {
    case SimAnn:
    {
        if (graspitEgPlanner) delete graspitEgPlanner;
        SimAnnPlanner * planner = new SimAnnPlanner(mHand);
        planner->setModelState(&graspPlanningState);
        graspitEgPlanner = planner;
        break;
    }
    /*case Loop: {
        if (graspitEgPlanner) delete graspitEgPlanner;
        graspitEgPlanner = new LoopEigenGraspPlanner(mHand);
        ((LoopEigenGraspPlanner*)graspitEgPlanner)->setModelState(&graspPlanningState);
    }
    case MultiThreaded: {
        if (graspitEgPlanner) delete graspitEgPlanner;
        graspitEgPlanner = new GuidedEigenGraspPlanner(mHand);
        ((GuidedEigenGraspPlanner*)graspitEgPlanner)->setModelState(&graspPlanningState);
    } */
    default:
    {
        PRINTERROR("Unknown planner type requested");
        return;
    }
    }

    plannerReset();
}



void EigenGraspPlanner::setPlanningParameters()
{
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    std::string _searchEnergyType = getSearchEnergyType(graspitSearchEnergyType);
    graspitEgPlanner->setEnergyType(_searchEnergyType);

    // contact type
    if (useContacts)
    {
        graspitEgPlanner->setContactType(CONTACT_PRESET);
        // if (mEnergyCalculator) mEnergyCalculator->setContactType(CONTACT_PRESET);
    }
    else
    {
        graspitEgPlanner->setContactType(CONTACT_LIVE);
        // if (mEnergyCalculator) mEnergyCalculator->setContactType(CONTACT_LIVE);
    }


    // if (mEnergyCalculator) mEnergyCalculator->setType(_searchEnergyType);
}




void EigenGraspPlanner::plannerReset()
{
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    assert(graspitEgPlanner);
    setPlanningParameters();

    graspitEgPlanner->resetPlanner();
}


void EigenGraspPlanner::plannerUpdate()
{
    PRINTMSG("=== EigenGraspPlanner update ===");
    updateResults();
}


void EigenGraspPlanner::plannerComplete()
{
    //printPlanningResults();
    PRINTMSG("=== EigenGraspPlanner complete ===");
}

void EigenGraspPlanner::updateResults()
{
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    int nStep = graspitEgPlanner->getCurrentStep();
    float runTime = graspitEgPlanner->getRunningTime();
    PRINTMSG("Current Step: " << nStep);
    // PRINTMSG("Current time: " << runTime);

    printResult(0, false);
}


/* double EigenGraspPlanner::getEnergy(const GraspPlanningState * s)
{
    s->execute();
    bool l; double e;
    if (mEnergyCalculator) mEnergyCalculator->analyzeCurrentPosture(s->getHand(), s->getObject(), l, e, false);
    return e;
}*/




/*void EigenGraspPlanner::printEnergy(int i)
{
    assert (i>=0 && i < graspitEgPlanner->getListSize());
    const GraspPlanningState *s = graspitEgPlanner->getGrasp(i);
    s->execute();
    bool l; double e;
    if (mEnergyCalculator) mEnergyCalculator->analyzeCurrentPosture(s->getHand(), s->getObject(), l, e, false);
    PRINTMSG("Re-computed energy: " << e);
}
*/

void EigenGraspPlanner::printResult(int i, bool detailed)
{
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    int d = graspitEgPlanner->getListSize();
    int rank, size, iteration;
    double energy;

    int displayState = i;

    if (d == 0)
    {
        displayState = rank = size = energy = iteration = 0;
    }
    else if (displayState < 0)
    {
        displayState = 0;
    }
    else if (displayState >= d)
    {
        displayState = d - 1;
    }

    const GraspPlanningState *s = NULL;

    if (d != 0)
    {
        s = graspitEgPlanner->getGrasp(displayState);
        if (!s)
        {
            PRINTERROR("GraspPlanningState is NULL!");
            return;
        }
        rank = displayState + 1;
        size = d;
        iteration = s->getItNumber();
        energy = s->getEnergy();
    }

    /*
    FILE *f = fopen("foo.txt","w");
    for (int i=0; i<graspitEgPlanner->getListSize(); i++) {
        for(int j=i+1; j<graspitEgPlanner->getListSize(); j++) {
            float d = graspitEgPlanner->getGrasp(i)->distance( graspitEgPlanner->getGrasp(j) );
            fprintf(stderr,"%d -- %d: %f\n",i+1,j+1,d);
        }
        fprintf(stderr,"\n");
        graspitEgPlanner->getGrasp(i)->writeToFile(f);
    }
    fclose(f);
    */

    // PRINTMSG("Rank: " << rank << "/" << size);
    // PRINTMSG("Iteration: " << iteration);
    PRINTMSG("Energy: " << energy);

    if (s && detailed)
    {
        PRINTMSG("Detailed state:");
        s->printState();
    }
}


void EigenGraspPlanner::printPlanningResults()
{
    PRINTMSG("########## Final results ############");
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }
    int numResults = graspitEgPlanner->getListSize();
    for (int i = 0; i < numResults; ++i)
    {
        PRINTMSG("---------------------");
        PRINTMSG("--- Result #" << (i + 1) << "  ---");
        PRINTMSG("---------------------");
        printResult(i, true);
        // printEnergy(i);
    }
}


/*
void EigenGraspPlanner::inputLoad(const std::string& inputGraspFile)
{
    if (inputGraspFile.empty())
    {
        PRINTERROR("No input grasp file specified");
        return;
    }

    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    FILE *fp = fopen(inputGraspFile.c_str(), "r");
    bool success = true;
    if (!fp)
    {
        PRINTMSG("Failed to open input file!");
        success = false;
    }
    else if (!graspitEgPlanner->getTargetState()->readFromFile(fp))
    {
        PRINTMSG("Failed to read target from input file!");
        success = false;
    }
    else
    {
        PRINTMSG("Target values loaded successfully");
    }
    fclose(fp);
    graspitEgPlanner->setInput(INPUT_FILE, success);
}
*/
