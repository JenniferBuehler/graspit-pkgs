#ifndef GRASP_PLANNING_GRASPIT_GRASPITDATABASEMANAGER_H
#define GRASP_PLANNING_GRASPIT_GRASPITDATABASEMANAGER_H

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


#include <grasp_planning_graspit/GraspItAccessor.h>
#include <string>
#include <vector>

namespace GraspIt
{

class GraspItSceneManager;

/**
 * \brief Superclass for all interfaces which want to access the GraspIt world for
 * database management purposes.
 *
 * This interface identifies robots and objects with names (strings) which have to be
 * unique among all robots and among all objects. The names identify a particular robot/object,
 * so several robots or objects of same type can be maintained with different names.
 *
 * When an object or robot is added to the database, they also get a uniqe model int ID which can be
 * used to access the model, independent of the fact whether it is a robot or an object.
 * The int id's are unique among *both* robots and objects.
 *
 * In ROS manipulation_msgs/GraspPlanning.srv, a body (object/obstacle) has an int ID
 * (see also household_objects_database_msgs/DatabaseModelPose.msg)
 * and a robot/hand has a string name. This interface allows to handle
 * both of these conventions. While robots will then also have an int id, and objects
 * also a string name, it is optional to the user of this interface which of the
 * identifiers to use.
 *
 * This interface allows to:
 * - add robots and objects to the database
 * - load them into the current %GraspIt world which is maintained by the GraspItSceneManager
 *      passed into this classes constructor.
 * - remove / "unload" objects from the current %GraspIt world again
 * - save the currently loaded graspit world as a file
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GraspItDatabaseManager: public GraspItAccessor
{
public:
    /**
     * \param name the name of this GraspItDatabaseManager object
     */
    GraspItDatabaseManager(const std::string& name, const SHARED_PTR<GraspItSceneManager>& interface):
        GraspItAccessor(name, interface)
    {
    }
    virtual ~GraspItDatabaseManager()
    {
    }

    /**
     * Loads robot (hand) from XML file and adds it to the database. See also GraspItSceneManager::loadRobot().
     * After it has been loaded to the database, it is open to the actual GraspItDatabaseManager implementation
     * wheter the robot is left in the graspit world, or removed from it.
     * \param robotName the name to store this robot with, and also the name to use for this robot in the
     *                  graspit world.
     * \param jointNames The URDF names of the joints in the order as they appear in the graspit file
     *          These are all joints that are involved in resulting grasps, so mostly these are only
     *          the fingers.
     * \retval >=0 success, and returns int ID of this model (model IDs are for robots and objects)
     * \retval -1 failed to load world with graspit source.
     * \retval -2 graspit not initialized.
     * \retval -3 file does not exist.
     * \retval -4 could not add robot to database because it already exists
     * \retval -5 no robot name specified
     * \retval -6 other error when adding the robot to the database.
     */
    virtual int loadRobotToDatabase(const std::string& filename, const std::string& robotName,
                                    const std::vector<std::string>& jointNames) = 0;

    /**
     * Loads an object from XML file and adds it to the database. See also GraspItSceneManager::loadObject().
     * After it has been loaded to the database, it is open to the actual GraspItDatabaseManager implementation
     * wheter the object is left in the graspit world, or removed from it.
     * \param objectName the name to store this object with, and also the name to use for this object in the
     *                   graspit world.
     * \retval >=0 success, and returns int ID of this model (model IDs are for robots and objects)
     * \retval -1 failed to load world with graspit source.
     * \retval -2 graspit not initialized.
     * \retval -3 file does not exist.
     * \retval -4 could not add object to database because it already exists
     * \retval -5 no object name specified
     * \retval -6 other error when adding the object to the database.
     */
    virtual int loadObjectToDatabase(const std::string& filename, const std::string& objectName,
                                     const bool asGraspable) = 0;

    /**
     * Loads this robot into the %GraspIt world
     * \retval 0 success
     * \retval -1 robot not in database
     * \retval -2 graspit not initialized or other error adding robot to graspIt
     */
    int loadRobotToWorld(const std::string& robotName, const EigenTransform& transform);


    /**
     * Loads this object into the %GraspIt world
     * \retval 0 success
     * \retval -1 object not in database
     * \retval -2 graspit not initialized or other error adding robot to graspIt
     */
    int loadObjectToWorld(const std::string& objectName, const EigenTransform& transform);


    /**
     * loads the model with this ID (returned by load*ToDatabase() functions) into the
     * graspit world
     * \retval 0 success
     * \retval -1 model not in database
     * \retval -2 graspit not initialized or other error adding robot to graspIt
     */
    int loadToWorld(const int modelID, const EigenTransform& transform);

    /**
     * Unloads this robot from the graspit world
     * \retval 0 success
     * \retval -1 robot not loaded in the world
     * \retval -2 robot not in the database
     * \retval -3 graspit not initialized or other error removing the robot from graspIt
     */
    int unloadRobotFromWorld(const std::string& robotName);

    /**
     * Unloads this object from the graspit world
     * \retval 0 success
     * \retval -1 object not loaded in the world
     * \retval -2 object not in the database
     * \retval -3 graspit not initialized or other error removing the object from graspit
     */
    int unloadObjectFromWorld(const std::string& objectName);

    /**
     * Unloads this robot or object from the graspit world
     * \retval 0 success
     * \retval -1 model not loaded in the world
     * \retval -2 model not in the database
     * \retval -3 graspit not initialized or other error removing the model from graspit
     */
    int unloadFromWorld(const int modelID);



    /**
     * Save the currently loaded world in the given file
     * \param asInventor if true, the world is going to be saved as inventor file.
     *                   Otherwise, it will be saved as $GraspIt XML world file.
     * \param createDir if true, the directory in which the file is to be saved is created if it does not exist.
     */
    bool saveLoadedWorld(const std::string& filename, const bool asInventor, const bool createDir = false);

    /**
     * Adds the names of all robots currently loaded in the %GraspIt world in the vector.
     * Does not clear the vector!
     */
    virtual void getAllLoadedRobotNames(std::vector<std::string>& names) const = 0;

    /**
     * Adds the ids of all robots currently loaded in the %GraspIt world in the vector
     * Does not clear the vector!
     */
    virtual void getAllLoadedRobotIDs(std::vector<int>& ids) const = 0;

    /**
     * Adds the names of all objects currently loaded in the %GraspIt world in the vector
     * Does not clear the vector!
     */
    virtual void getAllLoadedObjectNames(std::vector<std::string>& names) const = 0;

    /**
     * Adds the ids of all objects currently loaded in the %GraspIt world in the vector
     * Does not clear the vector!
     */
    virtual void getAllLoadedObjectIDs(std::vector<int>& ids) const = 0;

    /**
     * \param id the model ID of the robot with this name
     * \return false if this robot is not in the database.
     */
    virtual bool getRobotModelID(const std::string& robotName, int& id) const = 0;

    /**
     * \param id the model ID of the object with this name
     * \return false if this object is not in the database.
     */
    virtual bool getObjectModelID(const std::string& objectName, int& id) const = 0;

    /**
     * Checks if this model is currently loaded in the graspit world
     * \retval 1 model is loaded
     * \retval 0 model is not loaded
     * \retval -1 model ID could not be found in database
     */
    int isModelLoaded(const int modelID) const;

    /**
     * Returns the name and type of the model with this ID which is also used in the
     * %GraspIt world.
     * \param isRobot true if model is a robot, false if it is an object.
     */
    virtual bool getModelNameAndType(const int modelID, std::string& name, bool& isRobot) const = 0;


    /**
     * Returns the joint names for this robot as used in loadRobotToDatabase().
     * These are all joints that are involved in resulting grasps, so mostly these are only
     * the fingers.
     * \return false if this robot is not in the database.
     */
    virtual bool getRobotJointNames(const std::string& robotName,
                                    std::vector<std::string>& jointNames) const = 0;

protected:
    /**
     * Checks whether this model ID is a robot or an object
     * \retval 2 is an object
     * \retval 1 is a robot
     * \retval -1 does not exist
     */
    virtual int getModelType(const int modelID) const = 0;

    /**
     * Retrieves the Robot object from the database, if it exists in there.
     */
    virtual Robot * getRobotFromDatabase(const std::string& robotName) = 0;

    /**
     * Retrieves the Robot object from the database, if it exists in there.
     * \return return NULL if not in the database, or an object and not a robot.
     */
    virtual Robot * getRobotFromDatabase(const int modelID) = 0;

    /**
     * Retrieves the Body object from the database, if it exists in there.
     */
    virtual Body * getObjectFromDatabase(const std::string& objectName) = 0;

    /**
     * Retrieves the Body object from the database, if it exists in there.
     * \return return NULL if not in the database, or a robot and not an object.
     */
    virtual Body * getObjectFromDatabase(const int modelID) = 0;

    /**
     * This class is meant to remain abstract. Subclasses can decide whether they need access
     * to the inventor thread.
     */
    virtual void idleEventFromSceneManager() = 0;

    /**
     * This class is meant to remain abstract. Subclasses can decide whether they need access
     * to the inventor thread.
     */
    virtual void onSceneManagerShutdown() = 0;
};
}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_GRASPITDATABASEMANAGER_H
