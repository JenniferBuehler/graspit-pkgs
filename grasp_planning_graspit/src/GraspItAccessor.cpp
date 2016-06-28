/**
   Interface which allows subclasses to access the GraspIt world.

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


#include <grasp_planning_graspit/GraspItAccessor.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <string>

using GraspIt::GraspItAccessor;
using GraspIt::GraspItSceneManager;

GraspItAccessor::GraspItAccessor(const std::string& objectName, const SHARED_PTR<GraspItSceneManager>& interface):
    graspItInterface(interface),
    name(objectName),
    registered(false),
    scheduled(false)
{
}

GraspItAccessor::~GraspItAccessor()
{
    // std::cout<<"GraspItAccessor destructor."<<std::endl;
    if (registered)
    {
        PRINTERROR("Method GraspItAccessor::removeFromIdleListeners() has not been called by subclass destructor! This may lead to memory corruption.");
    }
}


std::string GraspItAccessor::getName() const
{
    return name;
}

bool GraspItAccessor::addAsIdleListener()
{
    registered = graspItInterface->addIdleListener(this);
}

bool GraspItAccessor::removeFromIdleListeners()
{
    registered = !graspItInterface->removeIdleListener(this);
}


void GraspItAccessor::scheduleForIdleEventUpdate()
{
    UNIQUE_LOCK(scheduledMtx);
    scheduled = true;
    graspItInterface->scheduleIdleEvent();
}

void GraspItAccessor::unschedule()
{
    UNIQUE_LOCK(scheduledMtx);
    scheduled = false;
}

bool GraspItAccessor::isScheduledForIdleEvent() const
{
    UNIQUE_LOCK(scheduledMtx);
    return scheduled;
}


const SHARED_PTR<GraspItSceneManager>& GraspItAccessor::getGraspItSceneManager()
{
    return graspItInterface;
}
const SHARED_PTR<const GraspItSceneManager> GraspItAccessor::readGraspItSceneManager() const
{
    return graspItInterface;
}

bool GraspItAccessor::eventThreadRunsQt() const
{
    return true;
}

bool GraspItAccessor::tryLockWorld()
{
    return graspItInterface->tryLockWorld();
}

void GraspItAccessor::lockWorld()
{
    graspItInterface->lockWorld();
}

void GraspItAccessor::unlockWorld()
{
    graspItInterface->unlockWorld();
}

UNIQUE_RECURSIVE_LOCK GraspItAccessor::getUniqueWorldLock()
{
    return graspItInterface->getUniqueWorldLock();
}

Hand * GraspItAccessor::getCurrentHand()
{
    return graspItInterface->getCurrentHand();
}

const Hand * GraspItAccessor::readCurrentHand() const
{
    return graspItInterface->readCurrentHand();
}

GraspableBody * GraspItAccessor::getCurrentGraspableBody()
{
    return graspItInterface->getCurrentGraspableBody();
}

const GraspableBody * GraspItAccessor::readCurrentGraspableBody() const
{
    return graspItInterface->readCurrentGraspableBody();
}

Robot * GraspItAccessor::getRobot(const std::string& name)
{
    return graspItInterface->getRobot(name);
}

Robot * GraspItAccessor::getRobot(const unsigned int i)
{
    return graspItInterface->getRobot(i);
}

GraspableBody * GraspItAccessor::getGraspableBody(const std::string& name)
{
    return graspItInterface->getGraspableBody(name);
}

GraspableBody * GraspItAccessor::getGraspableBody(const unsigned int i)
{
    return graspItInterface->getGraspableBody(i);
}

Body * GraspItAccessor::getBody(const std::string& name)
{
    return graspItInterface->getBody(name);
}

Body * GraspItAccessor::getBody(const unsigned int i)
{
    return graspItInterface->getBody(i);
}

bool GraspItAccessor::isRobotLoaded(const Robot * robot) const
{
    return graspItInterface->isRobotLoaded(robot);
}
bool GraspItAccessor::isObjectLoaded(const Body * object) const
{
    return graspItInterface->isObjectLoaded(object);
}


bool GraspItAccessor::removeElement(WorldElement* elem, const bool deleteInstance)
{
    return graspItInterface->removeElement(elem, deleteInstance);
}

int GraspItAccessor::addRobot(Robot* robot, const EigenTransform& worldTransform)
{
    return graspItInterface->addRobot(robot, worldTransform);
}

int GraspItAccessor::addBody(Body* body, const EigenTransform& worldTransform)
{
    return graspItInterface->addBody(body, worldTransform);
}
