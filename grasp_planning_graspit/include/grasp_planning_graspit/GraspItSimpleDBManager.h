#ifndef GRASP_PLANNING_GRASPIT_GRASPITSIMPLEDBMANAGER_H
#define GRASP_PLANNING_GRASPIT_GRASPITSIMPLEDBMANAGER_H

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


#include <grasp_planning_graspit/GraspItDatabaseManager.h>
#include <grasp_planning_graspit/ThreadImpl.h>
#include <map>
#include <utility>
#include <string>
#include <vector>

namespace GraspIt
{

class GraspItSceneManager;

/**
 * \brief Very simple implementation of GraspItDatabaseManager which just keeps
 * loaded objects in a std::map.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GraspItSimpleDBManager: public GraspItDatabaseManager
{
public:
    /**
     */
    GraspItSimpleDBManager(const std::string& objectName, const SHARED_PTR<GraspItSceneManager>& interface):
        GraspItDatabaseManager(objectName, interface),
        modelIdCounter(0)
    {
    }
    virtual ~GraspItSimpleDBManager()
    {
        // removeFromIdleListeners() is not required because addAsIdleListener() is definitely not called.
    }


    virtual int loadRobotToDatabase(const std::string& filename, const std::string& robotName,
                                    const std::vector<std::string>& jointNames);

    virtual int loadObjectToDatabase(const std::string& filename, const std::string& robotName, const bool asGraspable);

    virtual void getAllLoadedRobotNames(std::vector<std::string>& names) const;

    virtual void getAllLoadedRobotIDs(std::vector<int>& ids) const;

    virtual void getAllLoadedObjectNames(std::vector<std::string>& names) const;

    virtual void getAllLoadedObjectIDs(std::vector<int>& ids) const;

    virtual bool getRobotModelID(const std::string& robotName, int& id) const;

    virtual bool getObjectModelID(const std::string& objectName, int& id) const;

    virtual bool getModelNameAndType(const int modelID, std::string& name, bool& isRobot) const;

    virtual bool getRobotJointNames(const std::string& robotName,
                                    std::vector<std::string>& jointNames) const;

protected:
    /**
     * No need for getting updates from inventor thread.
     */
    virtual void idleEventFromSceneManager() {}
    virtual void onSceneManagerShutdown() {}

    virtual Robot * getRobotFromDatabase(const std::string& robotName);
    virtual Robot * getRobotFromDatabase(const int modelID);

    virtual Body * getObjectFromDatabase(const std::string& objectName);
    virtual Body * getObjectFromDatabase(const int modelID);

    virtual int getModelType(const int modelID) const;

private:
    // all bodies (objects and obstacles)
    std::map<std::string, Body*> objects;

    // all robots
    std::map<std::string, Robot*> robots;

    // all robots and boides names sorted by
    // model IDs. map Entry also contains flag on
    // wheter it is a robot (true) or object (false).
    typedef std::map<int, std::pair<std::string, bool> > ModelIdMap;

    // map which contains for each robot the list of URDF joint names in the
    // order they appear in the robots graspit description.
    typedef std::map<std::string, std::vector<std::string> > RobotJointNamesMap;

    ModelIdMap modelIDs;

    RobotJointNamesMap robotJointNames;

    // protects objects, robots, modelIDs and robotJointNames.
    // while any of the maps is changed, no others
    // may be accessed, to be on the safe side.
    RECURSIVE_MUTEX dbMtx;

    // counter for model IDs used for new objects/robots.
    int modelIdCounter;
};

}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_GRASPITSIMPLEDBMANAGER_H
