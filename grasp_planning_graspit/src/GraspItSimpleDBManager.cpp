/**
   Simple implementation of a GraspItDatabaseManager.

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


#include <grasp_planning_graspit/GraspItSimpleDBManager.h>
#include <grasp_planning_graspit/LogBinding.h>

#include <graspit/robot.h>

#include <map>
#include <string>
#include <vector>

using GraspIt::GraspItSimpleDBManager;

int GraspItSimpleDBManager::loadRobotToDatabase(const std::string& filename, const std::string& robotName,
        const std::vector<std::string>& jointNames)
{
    if (robotName.empty())
    {
        PRINTERROR("You have to specify a robot name");
        return -5;
    }

    UNIQUE_RECURSIVE_LOCK(dbMtx);

    std::map<std::string, Robot*>::iterator it = robots.find(robotName);
    if (it != robots.end())
    {
        PRINTERROR("Robot with name " << robotName << " already exists in the database.");
        return -4;
    }

    PRINTMSG("Loading robot");
    Robot * loadedRobot = NULL;
    {
        // the world object may not be changed while we briefly load the robot
        // and then remove it right after again. GraspItSceneManager does not support
        // yet to create a model without inserting it into the world at first.
        // This is because of methods called in the original graspit source.
        UNIQUE_RECURSIVE_LOCK lock = getUniqueWorldLock();

        int ret = getGraspItSceneManager()->loadRobot(filename, robotName);
        if (ret != 0)
        {
            PRINTERROR("Could not load robot " << robotName);
            return ret;
        }

        // PRINTMSG("Retrieving robot object");
        // get the Robot object
        loadedRobot = getRobot(robotName);

        // PRINTMSG("Removing element");
        // now, we'll remove the robot from the graspit world again, because we only need the
        // actual Robot object to store in the database
        if (!removeElement(loadedRobot, false))
        {
            PRINTERROR("FATAL: should have been able to remove the robot. System could now be insconsistent.");
            return -6;
        }
    }

//    PRINTMSG("Inserting to map");
    if (!robots.insert(std::make_pair(robotName, loadedRobot)).second)
    {
        PRINTERROR("Failed to insert robot into the map");
        return -6;
    }

    if (!robotJointNames.insert(std::make_pair(robotName, jointNames)).second)
    {
        PRINTERROR("Failed to insert robot joint names into the map");
        return -6;
    }

    ++modelIdCounter;
    modelIDs.insert(std::make_pair(modelIdCounter, std::make_pair(robotName, true)));

//    PRINTMSG("Successfully loaded robot to database.");
    return modelIdCounter;
}




int GraspItSimpleDBManager::loadObjectToDatabase(const std::string& filename,
        const std::string& objectName, const bool asGraspable)
{
    if (objectName.empty())
    {
        PRINTERROR("You have to specify an object name");
        return -5;
    }

    UNIQUE_RECURSIVE_LOCK(dbMtx);

    std::map<std::string, Body*>::iterator it = objects.find(objectName);
    if (it != objects.end())
    {
        PRINTERROR("Object with name " << objectName << " already exists in the database.");
        return -4;
    }

//    PRINTMSG("Loading object");
    Body * loadedObject = NULL;
    {
        // the world object may not be changed while we briefly load the robot
        // and then remove it right after again. GraspItSceneManager does not support
        // yet to create a model without inserting it into the world at first.
        // This is because of methods called in the original graspit source.
        UNIQUE_RECURSIVE_LOCK lock = getUniqueWorldLock();

        int ret = getGraspItSceneManager()->loadObject(filename, objectName, asGraspable);
        if (ret != 0)
        {
            PRINTERROR("Could not load object " << objectName);
            return ret;
        }

        // PRINTMSG("Retrieving object");
        // get the Object object
        loadedObject = getBody(objectName);

        // PRINTMSG("Removing element");
        // now, we'll remove the object from the graspit world again, because we only need the
        // actual Object object to store in the database
        if (!removeElement(loadedObject, false))
        {
            PRINTERROR("FATAL: should have been able to remove the object. System could now be insconsistent.");
            return -6;
        }
    }

//    PRINTMSG("Inserting to map");
    if (!objects.insert(std::make_pair(objectName, loadedObject)).second)
    {
        PRINTERROR("Failed to insert object into the map");
        return -6;
    }

    ++modelIdCounter;
    modelIDs.insert(std::make_pair(modelIdCounter, std::make_pair(objectName, false)));

//    PRINTMSG("Successfully loaded object to database.");
    return modelIdCounter;
}



bool GraspItSimpleDBManager::getRobotJointNames(const std::string& robotName,
        std::vector<std::string>& jointNames) const
{
    jointNames.clear();

    UNIQUE_RECURSIVE_LOCK(dbMtx);
    RobotJointNamesMap::const_iterator it = robotJointNames.find(robotName);
    if (it == robotJointNames.end())
    {
        PRINTERROR("Joints for robot '" << robotName << "' not found in database.");
        return false;
    }
    jointNames.insert(jointNames.begin(), it->second.begin(), it->second.end());
    return true;
}



Body * GraspItSimpleDBManager::getObjectFromDatabase(const std::string& objectName)
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::map<std::string, Body*>::iterator it = objects.find(objectName);
    if (it == objects.end())
    {
        PRINTERROR("Object with name " << objectName << " does not exists in the database.");
        return NULL;
    }
    return it->second;
}

Robot * GraspItSimpleDBManager::getRobotFromDatabase(const std::string& robotName)
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::map<std::string, Robot*>::iterator it = robots.find(robotName);
    if (it == robots.end())
    {
        PRINTERROR("Robot with name " << robotName << " does not exists in the database.");
        return NULL;
    }
    return it->second;
}


Body * GraspItSimpleDBManager::getObjectFromDatabase(const int modelID)
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::string name;
    bool isRobot;
    if (!getModelNameAndType(modelID, name, isRobot))
    {
        PRINTERROR("Robot/Object with model ID " << modelID << " not in database.");
        return NULL;
    }
    if (isRobot)
    {
        PRINTERROR("Model id " << modelID << " is a robot, not an object.");
        return NULL;
    }
    return getObjectFromDatabase(name);
}

Robot * GraspItSimpleDBManager::getRobotFromDatabase(const int modelID)
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::string name;
    bool isRobot;
    if (!getModelNameAndType(modelID, name, isRobot))
    {
        PRINTERROR("Robot/Object with model ID " << modelID << " not in database.");
        return NULL;
    }
    if (!isRobot)
    {
        PRINTERROR("Model id " << modelID << " is an object, not a robot.");
        return NULL;
    }
    return getRobotFromDatabase(name);
}




int GraspItSimpleDBManager::getModelType(const int modelID) const
{
    std::string name;
    bool isRobot;
    if (!getModelNameAndType(modelID, name, isRobot))
    {
        return -1;
    }
    // If robot, return 1.
    // For objects return 2.
    return isRobot ? 1 : 2;
}


bool GraspItSimpleDBManager::getModelNameAndType(const int modelID, std::string& name, bool& isRobot) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    ModelIdMap::const_iterator it = modelIDs.find(modelID);
    if (it == modelIDs.end())
    {
        return false;
    }
    name = it->second.first;
    isRobot = it->second.second;
    return true;
}



void GraspItSimpleDBManager::getAllLoadedRobotNames(std::vector<std::string>& names) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::map<std::string, Robot*>::const_iterator it;
    for (it = robots.begin(); it != robots.end(); ++it)
    {
        if (readGraspItSceneManager()->isRobotLoaded(it->first))
        {
            names.push_back(it->first);
        }
    }
}

void GraspItSimpleDBManager::getAllLoadedRobotIDs(std::vector<int>& ids) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    ModelIdMap::const_iterator it;
    for (it = modelIDs.begin(); it != modelIDs.end(); ++it)
    {
        bool isRobot = it->second.second;
        if (isRobot && readGraspItSceneManager()->isRobotLoaded(it->second.first))
        {
            ids.push_back(it->first);
        }
    }
}

void GraspItSimpleDBManager::getAllLoadedObjectNames(std::vector<std::string>& names) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    std::map<std::string, Body*>::const_iterator it;
    for (it = objects.begin(); it != objects.end(); ++it)
    {
        if (readGraspItSceneManager()->isObjectLoaded(it->first))
        {
            names.push_back(it->first);
        }
    }
}

void GraspItSimpleDBManager::getAllLoadedObjectIDs(std::vector<int>& ids) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    ModelIdMap::const_iterator it;
    for (it = modelIDs.begin(); it != modelIDs.end(); ++it)
    {
        bool isRobot = it->second.second;
        if (!isRobot && readGraspItSceneManager()->isRobotLoaded(it->second.first))
        {
            ids.push_back(it->first);
        }
    }
}



bool GraspItSimpleDBManager::getRobotModelID(const std::string& robotName, int& id) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    ModelIdMap::const_iterator it;
    for (it = modelIDs.begin(); it != modelIDs.end(); ++it)
    {
        bool isRobot = it->second.second;
        if (isRobot && (it->second.first == robotName))
        {
            id = it->first;
            return true;
        }
    }
    return false;
}

bool GraspItSimpleDBManager::getObjectModelID(const std::string& objectName, int& id) const
{
    UNIQUE_RECURSIVE_LOCK(dbMtx);
    ModelIdMap::const_iterator it;
    for (it = modelIDs.begin(); it != modelIDs.end(); ++it)
    {
        bool isRobot = it->second.second;
        if (!isRobot && (it->second.first == objectName))
        {
            id = it->first;
            return true;
        }
    }
    return false;
}

