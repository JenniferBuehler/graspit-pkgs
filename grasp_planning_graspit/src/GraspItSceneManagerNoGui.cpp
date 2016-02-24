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

#include <grasp_planning_graspit/GraspItSceneManagerNoGui.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/GraspItAccessor.h>

#include <string>
#include <vector>
#include <map>
#include <exception>

#include <QWidget>

#include <world.h>
#include <robot.h>
#include <body.h>
#include <grasp.h>
#include <ivmgr_nogui.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/sensors/SoIdleSensor.h>

#include <boost/filesystem.hpp>

using GraspIt::GraspItSceneManager;
using GraspIt::GraspItSceneManagerNoGui;
using GraspIt::Log;

GraspItSceneManagerNoGui::GraspItSceneManagerNoGui():
    ivThread(NULL),
    ivReady(false),
    mIdleSensor(NULL)
{
    initialize();
}

GraspItSceneManagerNoGui::~GraspItSceneManagerNoGui()
{
    shutdown();
}

void GraspItSceneManagerNoGui::initializeIVmgr()
{
    // start the thread loop which will kick off a new thread to run the QT Application
    ivThread = new THREAD_CONSTR(ivThreadLoop, this);

    // wait until Qt is initialized before doing anything else
    waitForInventorState(true);
}


void GraspItSceneManagerNoGui::destroyIVmgr()
{
    PRINTMSG("GraspItSceneManagerNoGui::destroyIVmgr()");
    // have to quit ivMgr main loop first so that the thread
    // exits its loop
    if (ivMgr)
    {
        IVmgrNoGui * _ivMgr = dynamic_cast<IVmgrNoGui*>(ivMgr);
        if (!_ivMgr)
        {
            throw std::string("Inconsistency:: ivMgr should be of class IVmgrNoGui!");
        }
        _ivMgr->exitMainLoop();
        waitForInventorState(false);
    }
    // PRINTMSG("Done in destroyIVmgr()");
    if (ivThread)
    {
        PRINTMSG("Now exit Inventor thread.");
        ivThread->join();
        delete ivThread;
        ivThread = NULL;
    }
    if (ivMgr)
    {
        delete ivMgr;
        ivMgr = NULL;
    }
}




World * GraspItSceneManagerNoGui::createGraspitWorld()
{
    if (!ivMgr)
    {
        throw std::string("Cannot initialize world without ivMgr begin intialized");
    }

    IVmgrNoGui * _ivMgr = dynamic_cast<IVmgrNoGui*>(ivMgr);
    if (!_ivMgr)
    {
        throw std::string("Inconsistency:: ivMgr should be of class IVmgrNoGui!");
    }

    PRINTMSG("Creating World");
    _ivMgr->createNewWorld("GraspItWorld");
    return _ivMgr->getWorld();
}



void GraspItSceneManagerNoGui::ivThreadLoop(GraspItSceneManagerNoGui * _this)
{
    PRINTMSG("Enter INVENTOR thread loop");

    IVmgrNoGui * ivMgrNoGui = new IVmgrNoGui("GraspIt");
    _this->ivMgr = ivMgrNoGui;

    _this->createIdleSensor();

    // we will schedule the idle sensor once, so that
    // it can call _this->setInventorReady(true)
    _this->mIdleSensor->schedule();

    // begin the main loop of inventor
    ivMgrNoGui->beginMainLoop();

    _this->deleteIdleSensor();

    _this->setInventorReady(false);

    PRINTMSG("Exit INVENTOR thread loop");
}


bool GraspItSceneManagerNoGui::isReady() const
{
    return isInventorReady();
}

void GraspItSceneManagerNoGui::waitUntilReady() const
{
    return waitForInventorState(true);
}

void GraspItSceneManagerNoGui::waitForInventorState(const bool value) const
{
    while (isInventorReady() != value) SLEEP(0.1);
}

bool GraspItSceneManagerNoGui::isInventorReady() const
{
    UNIQUE_LOCK lck(ivReadyMtx);
    return ivReady;
}

void GraspItSceneManagerNoGui::setInventorReady(const bool flag)
{
    UNIQUE_LOCK lck(ivReadyMtx);
    ivReady = flag;
}


void GraspItSceneManagerNoGui::deleteIdleSensor()
{
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
        mIdleSensor = NULL;
    }
}

void GraspItSceneManagerNoGui::createIdleSensor()
{
    if (mIdleSensor)
    {
        if (mIdleSensor->isScheduled()) mIdleSensor->unschedule();
        delete mIdleSensor;
    }

    // start the idle sensor which is triggered regularly from within the inventor thread
    mIdleSensor = new SoIdleSensor(sensorCB, this);
}


void GraspItSceneManagerNoGui::sensorCB(void *data, SoSensor *)
{
    // PRINTMSG(" ### sensorCB ###");
    GraspItSceneManagerNoGui* _this = dynamic_cast<GraspItSceneManagerNoGui*>(static_cast<GraspItSceneManagerNoGui*>(data));
    if (!_this)
    {
        PRINTERROR("Could not cast GraspItSceneManagerNoGui");
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


bool GraspItSceneManagerNoGui::scheduleIdleEvent()
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
