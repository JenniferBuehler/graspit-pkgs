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

#include <grasp_planning_graspit/EigenGraspPlannerNoQt.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/GraspItHelpers.h>

#include <string>
#include <vector>
#include <sstream>

#include <matvec3D.h>
#include <world.h>
#include <robot.h>
#include <body.h>
#include <grasp.h>
#include <eigenGrasp.h>
#include <EGPlanner/search.h>
#include <EGPlanner/searchState.h>
#include <EGPlanner/energy/searchEnergy.h>
#include <EGPlanner/egPlanner.h>
#include <EGPlanner/simAnn.h>
#include <EGPlanner/simAnnPlanner.h>
// #include <timeTest.h>
// #include <guidedPlanner.h>
// #include <loopPlanner.h>

#include <QWidget>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/sensors/SoIdleSensor.h>

#include <boost/filesystem.hpp>

#include <EGPlanner/search.h>

using GraspIt::EigenGraspPlannerNoQt;
using GraspIt::Log;

EigenGraspPlannerNoQt::EigenGraspPlannerNoQt(const std::string& name, const SHARED_PTR<GraspItSceneManager>& intr):
    GraspItAccessor(name, intr),
    graspitEgPlanner(NULL),
    graspitStateType(SPACE_AXIS_ANGLE),
    graspitSearchEnergyType("CONTACT_ENERGY"),
    useContacts(true)
{
    statusThread = new THREAD_CONSTR(statusThreadLoop, this);

    PRINTMSG("#######################################################");
//    addAsIdleListener();
}

EigenGraspPlannerNoQt::~EigenGraspPlannerNoQt()
{
    PRINTMSG("EigenGrasp planner destructor");

    removeFromIdleListeners();

    if (statusThread)
    {
        statusThread->detach();
        delete statusThread;
        statusThread = NULL;
    }

    graspitEgPlannerMtx.lock();
    if (graspitEgPlanner)
    {
        delete graspitEgPlanner;
        graspitEgPlanner = NULL;
    }
    graspitEgPlannerMtx.unlock();

    PRINTMSG("Exit EigenGrasp planner destructor");
}


void EigenGraspPlannerNoQt::onSceneManagerShutdown()
{
    if (statusThread)
    {
        statusThread->detach();
        delete statusThread;
        statusThread = NULL;
    }

    graspitEgPlannerMtx.lock();
    if (graspitEgPlanner)
    {
        delete graspitEgPlanner;
        graspitEgPlanner = NULL;
    }
    graspitEgPlannerMtx.unlock();
}

void EigenGraspPlannerNoQt::idleEventFromSceneManager()
{
}



void EigenGraspPlannerNoQt::statusThreadLoop(EigenGraspPlannerNoQt * _this)
{
    PRINTMSG("Enter STATUS thread loop");
    float slept = 0;
    while (true)
    {
        SLEEP(0.5);
        // PRINTMSG("Thread loop test");
        _this->graspitEgPlannerMtx.lock();
        if (_this->graspitEgPlanner)
        {
            if (_this->graspitEgPlanner->isActive())
            {
                _this->plannerUpdate();
            }
            else if (_this->graspitEgPlanner->isReady())
            {
                // this means it may have been paused, but becuse we don't have slots here,
                // we don't automatically get notified about the planning state being complete.
                _this->graspitEgPlanner->stopPlanner();
            }
        }
        _this->graspitEgPlannerMtx.unlock();
    }
    PRINTMSG("Exit STATUS thread loop");
}


bool EigenGraspPlannerNoQt::plan(const std::string& handName, const std::string& objectName,
                                 const EigenTransform * objectPose,
                                 const int maxPlanningSteps, const PlannerType& planType)
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

    return plan(maxPlanningSteps, planType);
}


bool EigenGraspPlannerNoQt::plan(const int maxPlanningSteps, const PlannerType& planType)
{
    if (!getGraspItSceneManager()->isInitialized())
    {
        PRINTERROR("Graspit scene manager not initialized. Cannot do planning.");
        return false;
    }

    // lock the world so that it won't change while we do the planning
    UNIQUE_RECURSIVE_LOCK lock = getUniqueWorldLock();

    PRINTMSG("Initializing planning...");
    if (!initPlanner(maxPlanningSteps, planType))
    {
        PRINTERROR("Could not initialize planner.");
        return false;
    }

    {
        UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
        if (!graspitEgPlanner)
        {
            PRINTERROR("EigenGraspPlannerNoQt not initialized");
            return false;
        }

        if (graspitEgPlanner->isActive())
        {
            PRINTERROR("EigenGraspPlannerNoQt is already active, you cannot call plan()");
            return false;
        }

        if (!graspitEgPlanner->isReady())
        {
            PRINTERROR("EigenGraspPlannerNoQt is not ready.");
            return false;
        }
    }

    PRINTMSG("Now initiating planning.");

    // to be consistent, the graspitEgPlanner object has to be protected by the mutex here as well.
    // on the other hand, this prevents the main planning loop from being interrupted. But
    // it is not designed to be interrupted anyway....
    // TODO at this point we cannot protect graspitEgPlanner object because otherwise the status
    // thread does not have access. Find a better solution for this!
    // graspitEgPlannerMtx.lock();
    graspitEgPlanner->runPlannerLoop();
    // graspitEgPlannerMtx.unlock();

    plannerComplete();
    PRINTMSG("Planning done.");

    // store results
    graspitEgPlannerMtx.lock();

    results.clear();
    int numGrasps = graspitEgPlanner->getListSize();
    for (int i = 0; i < numGrasps; ++i)
    {
        const GraspPlanningState *s = graspitEgPlanner->getGrasp(i);
        results.push_back(s);
    }
    graspitEgPlannerMtx.unlock();
    return true;
}

/*void EigenGraspPlannerNoQt::stopPlanner()
{
    // this only works if the QT method (asynchronous) is used
    graspitEgPlannerMtx.lock();
    graspitEgPlanner->pausePlanner();
    graspitEgPlannerMtx.unlock();
}*/

GraspIt::EigenTransform EigenGraspPlannerNoQt::getHandTransform(const GraspPlanningState * s) const
{
    const PositionState* handPosition = s->readPosition();
    EigenTransform handTransform = getEigenTransform(handPosition->getCoreTran());
    return handTransform;
}

GraspIt::EigenTransform EigenGraspPlannerNoQt::getObjectTransform(const GraspPlanningState * s) const
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
    Eigen::Quaterniond eObjectQuaternion(q.w, q.x, q.y, q.z);

    objectTransform = objectTransform.translate(eObjectTranslation);
    objectTransform = objectTransform.rotate(eObjectQuaternion);

    return objectTransform;
}


void EigenGraspPlannerNoQt::getGraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const
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
    for (int k = 0; k < numDOF; ++k)
    {
        dofs.push_back(_dofs[k]);
    }
}


void EigenGraspPlannerNoQt::getPregraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const
{
    GraspPlanningState sCopy(s);

    if (!sCopy.getHand())
    {
        PRINTERROR("Hand is NULL!");
        return;
    }
    
    // execute the auto-grasp in "opening" direction
    sCopy.execute();
    sCopy.getHand()->autoGrasp(false,-0.1,true);
    sCopy.saveCurrentHandState();

    const PostureState* handPosture = sCopy.readPosture();
    if (!handPosture)
    {
        PRINTERROR("Posture is NULL!");
        return;
    }

    // this gets the DOF values for the 3 fingers. If the hand has only one EigenGrasp which is symmetric,
    // the 3 values will be the same.
    const int numDOF = sCopy.getHand()->getNumDOF();
    double * _dofs = new double[numDOF];
    handPosture->getHandDOF(_dofs);
    for (int k = 0; k < numDOF; ++k)
    {
        dofs.push_back(_dofs[k]);
    }
}






void EigenGraspPlannerNoQt::getEigenGraspValues(const GraspPlanningState * s, std::vector<double>& egVals) const
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

bool EigenGraspPlannerNoQt::checkStateValidity(const GraspPlanningState * s) const
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


bool EigenGraspPlannerNoQt::saveResultsAsWorldFiles(const std::string& inDirectory,
        const std::string& fileNamePrefix, bool asGraspIt, bool asInventor, bool createDir)
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
        s->execute();

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
    }
    return true;
}


void EigenGraspPlannerNoQt::getResults(std::vector<EigenGraspResult>& allGrasps) const
{
    int numGrasps = results.size();

    for (int i = 0; i < numGrasps; ++i)
    {
        const GraspPlanningState * s = results[i];

        if (!checkStateValidity(s))
        {
            PRINTERROR("Cannot work with this state");
            continue;
        }

        EigenTransform handTransform = getHandTransform(s);
        // PRINTMSG("Hand transform: " << handTransform);

        EigenTransform objectTransform = getObjectTransform(s);
        // PRINTMSG("Object transform: " << objectTransform);

        // Compute hand transform relative to object
        // objectTransform * relTransform = handTransform
        // relTransform = objectTransform.inverse() * handTransform;
        EigenTransform relTransform = objectTransform.inverse() * handTransform;

        // PRINTMSG("Relative transform: " << relTransform);
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
        allGrasps.push_back(EigenGraspResult(relTransform, graspDOFs, pregraspDOFs, egVals,
                                             s->isLegal(), s->getEpsilonQuality(), s->getVolume(), s->getEnergy()));
    }
}

void EigenGraspPlannerNoQt::initSearchType(GraspPlanningState& graspPlanningState, const StateType& st)
{
    graspPlanningState.setPositionType(st);

    GraspableBody * mObject = getCurrentGraspableBody();
    if (!mObject)
    {
        PRINTERROR("Object is NULL!");
        return;
    }

    switch (st)
    {
    case SPACE_AXIS_ANGLE:
    case SPACE_COMPLETE:
    case SPACE_ELLIPSOID:
    {
        graspPlanningState.setRefTran(mObject->getTran());
        break;
    }
    case SPACE_APPROACH:
    {
        Hand * mHand = getCurrentHand();
        if (!mHand)
        {
            PRINTERROR("Hand is NULL!");
            return;
        }
        graspPlanningState.setRefTran(mHand->getTran());
        break;
    }
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




bool EigenGraspPlannerNoQt::initPlanner(const int maxPlanningSteps, const PlannerType& plannerType)
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
    graspPlanningState.setPositionType(graspitStateType);
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





void EigenGraspPlannerNoQt::initPlannerType(const GraspPlanningState& graspPlanningState, const PlannerType &pt)
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
        graspitEgPlanner = new LoopEigenGraspPlannerNoQt(mHand);
        ((LoopEigenGraspPlannerNoQt*)graspitEgPlanner)->setModelState(&graspPlanningState);
    }
    case MultiThreaded: {
        if (graspitEgPlanner) delete graspitEgPlanner;
        graspitEgPlanner = new GuidedEigenGraspPlannerNoQt(mHand);
        ((GuidedEigenGraspPlannerNoQt*)graspitEgPlanner)->setModelState(&graspPlanningState);
    } */
    default:
    {
        PRINTERROR("Unknown planner type requested");
        return;
    }
    }

    plannerReset();
}



void EigenGraspPlannerNoQt::setPlanningParameters()
{
    UNIQUE_RECURSIVE_LOCK(graspitEgPlannerMtx);
    if (!graspitEgPlanner)
    {
        PRINTERROR("Planner is NULL!");
        return;
    }

    graspitEgPlanner->setEnergyType(graspitSearchEnergyType);

    // contact type
    if (useContacts)
    {
        graspitEgPlanner->setContactType(CONTACT_PRESET);
    }
    else
    {
        graspitEgPlanner->setContactType(CONTACT_LIVE);
    }
}




void EigenGraspPlannerNoQt::plannerReset()
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

    // When using QT Signals/Slots, this function is indirectly called from
    // graspitEgPlanner->resetEigenGraspPlannerNoQt (last call here) via an emit.
    // So do it manually here.
    plannerUpdate();
}


void EigenGraspPlannerNoQt::plannerUpdate()
{
    PRINTMSG("=== EigenGraspPlannerNoQt update ===");
    updateResults();
}


void EigenGraspPlannerNoQt::plannerComplete()
{
    printPlanningResults();
}

void EigenGraspPlannerNoQt::updateResults()
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
    PRINTMSG("Current time: " << runTime);

    printResult(0, false);
}

void EigenGraspPlannerNoQt::printResult(int i, bool detailed)
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

    PRINTMSG("Rank: " << rank << "/" << size);
    PRINTMSG("Iteration: " << iteration);
    PRINTMSG("Energy: " << energy);

    if (s && detailed)
    {
        PRINTMSG("Detailed state:");
        s->printState();
    }
}


void EigenGraspPlannerNoQt::printPlanningResults()
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
void EigenGraspPlannerNoQt::inputLoad(const std::string& inputGraspFile)
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
