/**

   Interface to the GraspIt algorithms.

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

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>

#include <string>
#include <exception>

#include <graspit/graspitCore.h>

#include <Inventor/sensors/SoIdleSensor.h>

using GraspIt::GraspItSceneManagerHeadless;

GraspItSceneManagerHeadless::GraspItSceneManagerHeadless():
    ivThread(NULL),
    ivReady(false),
    mIdleSensor(NULL)
{
    initialize();
}

GraspItSceneManagerHeadless::~GraspItSceneManagerHeadless()
{
    shutdown();
}

void GraspItSceneManagerHeadless::initializeCore()
{
    // start the thread loop which will kick off a new thread to run the QT Application
    ivThread = new THREAD_CONSTR(ivThreadLoop, this);

    // wait until Qt is initialized before doing anything else
    waitForInventorState(true);
}


void GraspItSceneManagerHeadless::destroyCore()
{
    PRINTMSG("GraspItSceneManagerHeadless::destroyCore()");
    // have to quit core main loop first so that the thread
    // exits its loop
    /*if (core)
    {
        GraspitCore * _core = dynamic_cast<GraspitCore*>(core);
        if (!_core)
        {
            throw std::string("Inconsistency:: core should be of class GraspitCore!");
        }
        _core->exitMainLoop();
        waitForInventorState(false);
    }*/
    core->exitMainLoop();
    // PRINTMSG("Done in destroyCore()");
    if (ivThread)
    {
        PRINTMSG("Now exit Inventor thread.");
        ivThread->join();
        delete ivThread;
        ivThread = NULL;
    }
    if (core)
    {
        delete core;
        core = NULL;
    }
}




World * GraspItSceneManagerHeadless::createNewGraspitWorld()
{
    if (!core)
    {
        throw std::string("Cannot initialize world without core begin intialized");
    }

    PRINTMSG("Creating new graspit world");
    core->emptyWorld("GraspItWorld");
    return core->getWorld();
}



void GraspItSceneManagerHeadless::ivThreadLoop(GraspItSceneManagerHeadless * _this)
{
    PRINTMSG("Enter INVENTOR thread loop");
    std::string name("GraspIt");
    std::string hArg("--headless");
    char * args[2];
    args[0]=const_cast<char*>(name.c_str());
    args[1]=const_cast<char*>(hArg.c_str());
    PRINTMSG("Starting with args "<<args[0]<<", "<<args[1]);
    GraspitCore * coreHeadless = new GraspitCore(2,args);
    PRINTMSG("Created.");
    _this->core = coreHeadless;

    _this->createIdleSensor();

    // we will schedule the idle sensor once, so that
    // it can call _this->setInventorReady(true)
    _this->mIdleSensor->schedule();

    // begin the main loop of inventor
    coreHeadless->startMainLoop();

    _this->deleteIdleSensor();

    _this->setInventorReady(false);

    PRINTMSG("Exit INVENTOR thread loop");
}


bool GraspItSceneManagerHeadless::isReady() const
{
    return isInventorReady();
}

void GraspItSceneManagerHeadless::waitUntilReady() const
{
    return waitForInventorState(true);
}

void GraspItSceneManagerHeadless::waitForInventorState(const bool value) const
{
    while (isInventorReady() != value) SLEEP(0.1);
}

bool GraspItSceneManagerHeadless::isInventorReady() const
{
    UNIQUE_LOCK lck(ivReadyMtx);
    return ivReady;
}

void GraspItSceneManagerHeadless::setInventorReady(const bool flag)
{
    UNIQUE_LOCK lck(ivReadyMtx);
    ivReady = flag;
}


void GraspItSceneManagerHeadless::deleteIdleSensor()
{
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
}

void GraspItSceneManagerHeadless::createIdleSensor()
{
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
    }

    // start the idle sensor which is triggered regularly from within the inventor thread
    mIdleSensor = new SoIdleSensor(sensorCB, this);
}


void GraspItSceneManagerHeadless::sensorCB(void *data, SoSensor *)
{
    // PRINTMSG(" ### sensorCB ###");
    GraspItSceneManagerHeadless* _this = dynamic_cast<GraspItSceneManagerHeadless*>(static_cast<GraspItSceneManagerHeadless*>(data));
    if (!_this)
    {
        PRINTERROR("Could not cast GraspItSceneManagerHeadless");
        return;
    }

    // Inventor obviously is ready, because otherwise this callback would not have been called.
    // use this here to trigger first initialization of the ivReady variable.
    if (!_this->isInventorReady())
    {
        _this->setInventorReady(true);
    }

    _this->processIdleEvent();
}


bool GraspItSceneManagerHeadless::scheduleIdleEvent()
{
    // PRINTMSG("Registering");
    if (!isInventorReady())
    {
        PRINTERROR("Cannot schedule update because Inventor is not initialized yet");
        return false;
    }

    assert(mIdleSensor);

    // trigger another call of this method in the next event loop iteration
    if (!mIdleSensor->isScheduled())
    {
        mIdleSensor->schedule();
    }
    return true;
}
