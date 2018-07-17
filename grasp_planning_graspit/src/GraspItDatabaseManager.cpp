/**
   Interface for accessing the GraspIt world for database management purposes.

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


#include <grasp_planning_graspit/GraspItDatabaseManager.h>
#include <grasp_planning_graspit/LogBinding.h>

#include <graspit/robot.h>

#include <string>

using GraspIt::GraspItDatabaseManager;

int GraspItDatabaseManager::loadRobotToWorld(const std::string& robotName, const EigenTransform& transform)
{
    Robot * robot = getRobotFromDatabase(robotName);
    if (!robot)
    {
        PRINTERROR("Robot " << robotName << " does not exist in database");
        return -1;
    }

    PRINTMSG("Adding robot...");
    int ret = addRobot(robot, transform);
    if (ret != 0)
    {
        PRINTERROR("Could not add robot to GraspIt world. Error code " << ret);
        return -2;
    }
    return 0;
}

int GraspItDatabaseManager::loadObjectToWorld(const std::string& objectName, const EigenTransform& transform)
{
    Body * object = getObjectFromDatabase(objectName);
    if (!object)
    {
        PRINTERROR("Object " << objectName << " does not exist in database");
        return -1;
    }

    PRINTMSG("Adding object...");
    int ret = addBody(object, transform);
    if (ret != 0)
    {
        PRINTERROR("Could not add object to GraspIt world. Error code " << ret);
        return -2;
    }
    return 0;
}

int GraspItDatabaseManager::loadToWorld(const int modelID, const EigenTransform& transform)
{
    int mType = getModelType(modelID);
    if (mType < 0)
    {
        PRINTERROR("Model " << modelID << " does not exist in database.");
        return -1;
    }
    if (mType == 1)  // is a robot
    {
        Robot * robot = getRobotFromDatabase(modelID);
        if (!robot)
        {
            PRINTERROR("Robot ID=" << modelID << " could not be retrieved.");
            return -1;
        }

        // PRINTMSG("Adding robot...");
        int ret = addRobot(robot, transform);
        if (ret != 0)
        {
            PRINTERROR("Could not add robot to GraspIt world. Error code " << ret);
            return -2;
        }
    }
    else     // is an object
    {
        Body * object = getObjectFromDatabase(modelID);
        if (!object)
        {
            PRINTERROR("Object ID=" << modelID << " could not be retrieved.");
            return -1;
        }

        // PRINTMSG("Adding object...");
        int ret = addBody(object, transform);
        if (ret != 0)
        {
            PRINTERROR("Could not add object to GraspIt world. Error code " << ret);
            return -2;
        }
    }
    return 0;
}


int GraspItDatabaseManager::unloadRobotFromWorld(const std::string& robotName)
{
    int id = -1;
    if (!getRobotModelID(robotName, id))
    {
        PRINTERROR("Robot " << robotName << " does not exist in database.");
        return -2;
    }
    return unloadFromWorld(id);
}

int GraspItDatabaseManager::unloadObjectFromWorld(const std::string& objectName)
{
    int id = -1;
    if (!getObjectModelID(objectName, id))
    {
        PRINTERROR("Object " << objectName << " does not exist in database.");
        return -2;
    }
    return unloadFromWorld(id);
}

int GraspItDatabaseManager::unloadFromWorld(const int modelID)
{
    int mType = getModelType(modelID);
    if (mType < 0)
    {
        PRINTERROR("Model " << modelID << " does not exist in database.");
        return -2;
    }
    WorldElement * elem = NULL;
    if (mType == 1)  // is a robot
    {
        Robot * robot = getRobotFromDatabase(modelID);
        if (!robot)
        {
            PRINTERROR("Robot ID=" << modelID << " could not be retrieved from the database.");
            return -2;
        }
        if (!isRobotLoaded(robot))
        {
            PRINTMSG("Robot " << modelID << " is not loaded GraspIt world.");
            return -1;
        }
        elem = robot;
        PRINTMSG("Removing robot " << modelID);
    }
    else   // is an object
    {
        Body * object = getObjectFromDatabase(modelID);
        if (!object)
        {
            PRINTERROR("Object ID=" << modelID << " could not be retrieved from the database.");
            return -2;
        }
        if (!isObjectLoaded(object))
        {
            PRINTMSG("Object " << modelID << " is not loaded GraspIt world.");
            return -1;
        }
        elem = object;
        PRINTMSG("Removing object " << modelID);
    }

    if (!removeElement(elem, false))
    {
        PRINTERROR("Could not remove model " << modelID << " from GraspIt world.");
        return -3;
    }
    return 0;
}





bool GraspItDatabaseManager::saveLoadedWorld(const std::string& filename, const bool asInventor, const bool createDir)
{
    if (asInventor)
    {
        if (!getGraspItSceneManager()->saveInventorWorld(filename, createDir))
        {
            PRINTERROR("Could not save inventor world in " << filename);
            if (!createDir) PRINTERROR("Does the directory exist?");
            return false;
        }
        // PRINTMSG("Saved inventor world in "<<filename);
    }
    else
    {
        if (!getGraspItSceneManager()->saveGraspItWorld(filename, createDir))
        {
            PRINTERROR("Could not save graspit world in " << filename);
            if (!createDir) PRINTERROR("Does the directory exist?");
            return false;
        }
        // PRINTMSG("Saved graspit world in "<<filename);
    }
    return true;
}


/**
 * Checks if this model is currently loaded in the graspit world
 * \retval 1 model is loaded
 * \retval 0 model is not loaded
 * \retval -1 model ID could not be found in database
 */
int GraspItDatabaseManager::isModelLoaded(const int modelID) const
{
    std::string name;
    bool isRobot;
    if (!getModelNameAndType(modelID, name, isRobot))
    {
        PRINTERROR("Could not find model " << modelID << " in database");
    }
    if (isRobot)
    {
        return readGraspItSceneManager()->isRobotLoaded(name) ? 1 : 0;
    }
    return readGraspItSceneManager()->isObjectLoaded(name) ? 1 : 0;
}

