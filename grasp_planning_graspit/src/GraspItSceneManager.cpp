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

#include <grasp_planning_graspit/GraspItSceneManager.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/GraspItAccessor.h>
#include <grasp_planning_graspit/GraspItHelpers.h>

#include <string>
#include <vector>
#include <map>
#include <exception>

#include <graspit/world.h>
#include <graspit/robot.h>
#include <graspit/body.h>
#include <graspit/grasp.h>
#include <graspit/graspitCore.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/actions/SoWriteAction.h>
// #include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <boost/filesystem.hpp>

using GraspIt::GraspItSceneManager;

bool fileExists(const std::string& filename)
{
    return boost::filesystem::exists(filename) && boost::filesystem::is_regular_file(filename);
}

bool makeDirectoryIfNeeded(const std::string& dPath)
{
    try
    {
        boost::filesystem::path dir(dPath);
        boost::filesystem::path buildPath;

        for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end()); it != it_end; ++it)
        {
            buildPath /= *it;
            // std::cout << buildPath << std::endl;

            if (!boost::filesystem::exists(buildPath) &&
                    !boost::filesystem::create_directory(buildPath))
            {
                PRINTERROR("Could not create directory " << buildPath);
                return false;
            }
        }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        PRINTERROR(ex.what());
        return false;
    }
    return true;
}



GraspItSceneManager::GraspItSceneManager():
    graspitWorld(NULL),
    core(NULL),
    initialized(false),
    fakeQObjectParent(NULL)
{
}


GraspItSceneManager::~GraspItSceneManager()
{
    PRINTMSG("GraspItSceneManager destructor");

    if (core)
    {
        PRINTERROR("The IVmgr should have been deleted, either by calling shutdown(), or by subclasses destructor!");
        throw std::string("The IVmgr should have been deleted, either by calling shutdown(), or by subclasses destructor!");
    }

    if (fakeQObjectParent)
    {
        delete fakeQObjectParent;
        fakeQObjectParent = NULL;
    }
}

void GraspItSceneManager::initialize()
{
    if (initialized)
    {
        PRINTMSG("GraspItSceneManager already initialized.");
        return;
    }

    initializeCore();
    if (!core)
    {
        throw std::string("Cannot initialize world without core begin intialized");
    }

    fakeQObjectParent = new QObject();

    UNIQUE_RECURSIVE_LOCK lock(graspitWorldMtx);
    graspitWorld = createNewGraspitWorld();
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world was initialized to NULL");
        throw std::string("Graspit world was initialized to NULL");
    }

    waitUntilReady();

    PRINTMSG("Initialized GraspItSceneManager.");

    initialized = true;
}

void GraspItSceneManager::shutdown()
{
    if (!initialized)
    {
        PRINTMSG("GraspItSceneManager already shut down.");
        return;
    }
    initialized = false;

    // notify all registered accessors that the manager is
    // shutting down.
    registeredAccessorsMtx.lock();
    std::map<std::string, GraspItAccessor*>::iterator it;
    for (it = registeredAccessors.begin(); it != registeredAccessors.end(); ++it)
    {
        GraspItAccessor * s = it->second;
        s->onSceneManagerShutdown();
    }
    registeredAccessorsMtx.unlock();

    // destroy core
    destroyCore();

    if (core)
    {
        PRINTERROR("The IVmgr should have been deleted, either by calling shutdown(), or by subclasses destructor!");
        throw std::string("The IVmgr should have been deleted, either by calling shutdown(), or by subclasses destructor!");
    }

    if (fakeQObjectParent)
    {
        delete fakeQObjectParent;
        fakeQObjectParent = NULL;
    }
}



bool GraspItSceneManager::isInitialized() const
{
    return initialized && isReady();
}


bool GraspItSceneManager::removeIdleListener(GraspItAccessor* s)
{
    registeredAccessorsMtx.lock();
    PRINTMSG("Unregistering " << s->getName());
    std::map<std::string, GraspItAccessor*>::iterator it = registeredAccessors.find(s->getName());
    bool success = true;
    if (it == registeredAccessors.end())
    {
        PRINTMSG("INFO: Did not remove " << s->getName() << " from registered accessors because it wasn't registered");
        success = false;
    }
    else
    {
        registeredAccessors.erase(it);
    }
    registeredAccessorsMtx.unlock();
    return success;
}


bool GraspItSceneManager::addIdleListener(GraspItAccessor* s)
{
    PRINTMSG("Registering " << s->getName());
    registeredAccessorsMtx.lock();
    bool success = registeredAccessors.insert(std::make_pair(s->getName(), s)).second;
    registeredAccessorsMtx.unlock();
    return success;
}



void GraspItSceneManager::processIdleEvent()
{
    registeredAccessorsMtx.lock();
    // PRINTMSG("LOOP: Notify "<<_registeredAccessors.size());
    std::map<std::string, GraspItAccessor*>::iterator it;
    for (it = registeredAccessors.begin(); it != registeredAccessors.end(); ++it)
    {
        GraspItAccessor * s = it->second;
        if (s->isScheduledForIdleEvent())
        {
            s->idleEventFromSceneManager();
        }
    }
    registeredAccessorsMtx.unlock();
}



/*
Old method to get camera parameters. Is now done in new GraspitCore class.

 **
 * Computes camera parameters to set for watching the current scene.
 * This is needed for writing a world file with meaningful camera parameters.
 *
 * Will set the camera at twice the scene's diameter (diameter of whole bounding box) distance
 * away from the scene center along the x axis. It look at the scene along the x axis.
 * The focal distance is also the distance from the camera to the scene center to keep things simple.
 *
void GraspItSceneManager::getCameraParameters(Eigen::Vector3d & camPos, Eigen::Quaterniond& camQuat, double & fd) const
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return;
    }
    // make a fake viewport since we have no GUI here
    SbViewportRegion vpReg;
    vpReg.setWindowSize(1920, 1080);

    // this action can be used to compute a bounding box of the scene
    SoGetBoundingBoxAction bboxAction(vpReg);
    graspitWorldMtx.lock();
    bboxAction.apply(graspitWorld->getIVRoot());
    graspitWorldMtx.unlock();

    // get bounding box parameters.
    SbBox3f bbox = bboxAction.getBoundingBox();
    SbVec3f min = bbox.getMin();
    SbVec3f max = bbox.getMax();
    SbVec3f center = bbox.getCenter();
    SbVec3f diameter = max - min;
    float cx, cy, cz;
    center.getValue(cx, cy, cz);
    float diamLen = diameter.length();
    // PRINTMSG("Bounding box: center="<<cx<<"/"<<cy<<"/"<<cz<<" diam="<<diamLen);

    // camera position is along the negative x axis 3 times the length of the diamenter from the bounding box center
    camPos = Eigen::Vector3d(cx - diamLen, cy, cz);
    // camera orientation is looking along positive x axis
    camQuat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-1, 0, 0));
    fd = diamLen;
}
*/

bool GraspItSceneManager::saveGraspItWorld(const std::string& filename, bool createDir)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return false;
    }

    try
    {
        if (createDir && !makeDirectoryIfNeeded(getFileDirectory(filename)))
        {
            PRINTERROR("Could not create directory for file " << filename);
            return false;
        }
    }
    catch (int e)
    {
        PRINTERROR("An exception ocurred when trying to create the directory. Exception number " << e);
        return false;
    }

/* Old code using obsolete getCameraParameters(). This is now done in GraspitCore.
    // set the camera so that the world file can be written with some correct camera parameters
    // if no camera parameters are set, the world file can't be opened with the original simulatur GUI.
    Eigen::Vector3d camPos;
    Eigen::Quaterniond camQuat;
    double fd;
    getCameraParameters(camPos, camQuat, fd);
    // core->setCamera(camPos.x(), camPos.y(), camPos.z(),
    //                camQuat.x(), camQuat.y(), camQuat.z(), camQuat.w(), fd);
*/

    if (graspitWorld->save(filename.c_str()) == FAILURE)
    {
        PRINTERROR("GraspIt could not save world file " << filename);
        return false;
    }
    return true;
}


bool GraspItSceneManager::saveInventorWorld(const std::string& filename, bool createDir)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return false;
    }
    try
    {
        if (createDir && !makeDirectoryIfNeeded(getFileDirectory(filename)))
        {
            PRINTERROR("Could not create directory for file " << filename);
            return false;
        }
    }
    catch (int e)
    {
        PRINTERROR("An exception ocurred when trying to create the directory. Exception number " << e);
        return false;
    }

    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    graspitWorldMtx.lock();
    write.apply(graspitWorld->getIVRoot());
    graspitWorldMtx.unlock();
    write.getOutput()->closeFile();
    return true;
}



bool GraspItSceneManager::saveRobotAsInventor(const std::string& filename, const std::string& robotName,
                                   const bool createDir, const bool forceWrite)
{
    if (!forceWrite && fileExists(filename))
    {
        PRINTERROR("File " << filename << " already exists");
        return false;
    }
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return false;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot load " << filename << " with no initialized graspitWorld");
        return false;
    }

    // Check that no object with same name exists
    Robot * existingRobot = getRobotNoCheck(robotName);
    if (!existingRobot)
    {
        PRINTERROR("Robot with name " << robotName << " does not exist in world.");
        return false;
    }

    try
    {
        if (createDir && !makeDirectoryIfNeeded(getFileDirectory(filename)))
        {
            PRINTERROR("Could not create directory for file " << filename);
            return false;
        }
    }
    catch (int e)
    {
        PRINTERROR("An exception ocurred when trying to create the directory. Exception number " << e);
        return false;
    }

    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    write.apply(existingRobot->getIVRoot());
    write.getOutput()->closeFile();

    PRINTMSG("Saved robot IV to " << filename);
    return true;
}


bool GraspItSceneManager::saveObjectAsInventor(const std::string& filename, const std::string& name,
                                    const bool createDir, const bool forceWrite)
{
    if (name.empty())
    {
        PRINTERROR("Cannot save an object without a name");
        return false;
    }

    if (!forceWrite && fileExists(filename))
    {
        PRINTERROR("File " << filename << " already exists");
        return false;
    }
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return false;
    }

    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot load " << filename << " with no initialized graspitWorld");
        return false;
    }

    // Check that no object with same name exists
    Body * existingBody = getBodyNoCheck(name);

    if (!existingBody)
    {
        PRINTERROR("Body with name " << name << " is not loaded in world.");
        return false;
    }

    try
    {
        if (createDir && !makeDirectoryIfNeeded(getFileDirectory(filename)))
        {
            PRINTERROR("Could not create directory for file " << filename);
            return false;
        }
    }
    catch (int e)
    {
        PRINTERROR("An exception ocurred when trying to create the directory. Exception number " << e);
        return false;
    }

    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    write.apply(existingBody->getIVRoot());
    write.getOutput()->closeFile();

    PRINTMSG("Saved object IV to " << filename);
    return true;
}


std::vector<std::string> GraspItSceneManager::getRobotNames() const
{
    std::vector<std::string> names;
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int numR =  graspitWorld->getNumRobots();
    for (int i = 0; i < numR; ++i)
    {
        const Robot * r = graspitWorld->getRobot(i);
        names.push_back(r->getName().toStdString());
    }
    return names; 
}

std::vector<std::string> GraspItSceneManager::getObjectNames(bool graspable) const
{
    std::vector<std::string> names;
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    int numB =  graspable ? graspitWorld->getNumGB() : graspitWorld->getNumBodies();
    for (int i = 0; i < numB; ++i)
    {
        if (graspable)
        {
            GraspableBody * b = graspitWorld->getGB(i);
            names.push_back(b->getName().toStdString());
        } 
        else
        {
            Body * b = graspitWorld->getBody(i);
            names.push_back(b->getName().toStdString());
        }
    }
    return names; 
}




int GraspItSceneManager::loadWorld(const std::string& filename)
{
    if (!fileExists(filename))
    {
        PRINTERROR("File " << filename << " does not exist");
        return -3;
    }
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot load " << filename << " with no initialized graspitWorld");
        return -2;
    }

    PRINTMSG("Loading graspitWorld " << filename);
    if (graspitWorld->load(QString(filename.c_str())) == FAILURE)
    {
        PRINTERROR("Could not load graspitWorld " << filename);
        return -1;
    }
    PRINTMSG("Loaded graspitWorld " << filename);
    return 0;
}


int GraspItSceneManager::loadRobot(const std::string& filename, const std::string& robotName,
                                   const EigenTransform& worldTransform)
{
    if (!fileExists(filename))
    {
        PRINTERROR("File " << filename << " does not exist");
        return -3;
    }
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot load " << filename << " with no initialized graspitWorld");
        return -2;
    }

    // Check that no object with same name exists
    Robot * existingRobot = getRobotNoCheck(robotName);
    if (existingRobot)
    {
        PRINTERROR("Robot with name " << robotName << " already exists in world.");
        return -4;
    }

    Robot * robot = graspitWorld->importRobot(QString(filename.c_str()));
    if (!robot)
    {
        PRINTERROR("Could not import robot from " << filename);
        return -1;
    }

    robot->setName(QString(robotName.c_str()));

    transf trans = getGraspitTransform(worldTransform);
    robot->setTran(trans);

    PRINTMSG("Loaded robot " << filename);
    return 0;
}


int GraspItSceneManager::loadObject(const std::string& filename, const std::string& name,
                                    const bool asGraspable, const EigenTransform& worldTransform)
{
    if (name.empty())
    {
        PRINTERROR("Cannot load an object without a name");
        return -5;
    }

    if (!fileExists(filename))
    {
        PRINTERROR("File " << filename << " does not exist");
        return -3;
    }
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }

    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot load " << filename << " with no initialized graspitWorld");
        return -2;
    }

    // Check that no object with same name exists
    Body * existingBody = getBodyNoCheck(name);

    if (existingBody)
    {
        PRINTERROR("Body with name " << name << " already exists in world.");
        return -4;
    }

    Body * object = NULL;
    if (asGraspable)
    {
        object = graspitWorld->importBody("GraspableBody", QString(filename.c_str()));
    }
    else
    {
        object = graspitWorld->importBody("Body", QString(filename.c_str()));
    }
    if (!object)
    {
        PRINTERROR("Could not import object from " << filename);
        return -1;
    }

    object->setName(QString(name.c_str()));

    transf trans = getGraspitTransform(worldTransform);
    object->setTran(trans);

    PRINTMSG("Loaded object from " << filename);
    return 0;
}


GraspableBody * GraspItSceneManager::getGraspableBodyNoCheck(const std::string& name)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int gb =  graspitWorld->getNumGB();
    for (int i = 0; i < gb; ++i)
    {
        GraspableBody * b = graspitWorld->getGB(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}

const GraspableBody * GraspItSceneManager::readGraspableBodyNoCheck(const std::string& name) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int gb =  graspitWorld->getNumGB();
    for (int i = 0; i < gb; ++i)
    {
        GraspableBody * b = graspitWorld->getGB(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}



GraspableBody * GraspItSceneManager::getGraspableBody(const std::string& name)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getGraspableBodyNoCheck(name);
}


const GraspableBody * GraspItSceneManager::readGraspableBody(const std::string& name) const
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return readGraspableBodyNoCheck(name);
}



GraspableBody * GraspItSceneManager::getGraspableBodyNoCheck(const unsigned int i)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int gb =  graspitWorld->getNumGB();
    if (i > gb)
    {
        PRINTERROR("There is no " << i << "th graspable body");
        return NULL;
    }
    return graspitWorld->getGB(i);
}


GraspableBody * GraspItSceneManager::getGraspableBody(const unsigned int i)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getGraspableBodyNoCheck(i);
}



Body * GraspItSceneManager::getBodyNoCheck(const std::string& name)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int nb =  graspitWorld->getNumBodies();
    for (int i = 0; i < nb; ++i)
    {
        Body * b = graspitWorld->getBody(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}

const Body * GraspItSceneManager::readBodyNoCheck(const std::string& name) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int nb =  graspitWorld->getNumBodies();
    for (int i = 0; i < nb; ++i)
    {
        const Body * b = graspitWorld->getBody(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}



Body * GraspItSceneManager::getBody(const std::string& name)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getBodyNoCheck(name);
}


const Body * GraspItSceneManager::readBody(const std::string& name) const
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return readBodyNoCheck(name);
}



Body * GraspItSceneManager::getBodyNoCheck(const unsigned int i)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int nb =  graspitWorld->getNumBodies();
    if (i > nb)
    {
        PRINTERROR("There is no " << i << "th body");
        return NULL;
    }
    return graspitWorld->getBody(i);
}


Body * GraspItSceneManager::getBody(const unsigned int i)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getBodyNoCheck(i);
}




const Robot * GraspItSceneManager::readRobotNoCheck(const std::string& name) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int numR =  graspitWorld->getNumRobots();
    for (int i = 0; i < numR; ++i)
    {
        const Robot * b = graspitWorld->getRobot(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}




Robot * GraspItSceneManager::getRobotNoCheck(const std::string& name)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int numR =  graspitWorld->getNumRobots();
    for (int i = 0; i < numR; ++i)
    {
        Robot * b = graspitWorld->getRobot(i);
        if (b->getName().toStdString() == name)
        {
            return b;
        }
    }
    return NULL;
}

Robot * GraspItSceneManager::getRobot(const std::string& name)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getRobotNoCheck(name);
}


const Robot * GraspItSceneManager::readRobot(const std::string& name) const
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return readRobotNoCheck(name);
}



Robot * GraspItSceneManager::getRobotNoCheck(const unsigned int i)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    int numR =  graspitWorld->getNumRobots();
    if (i > numR)
    {
        PRINTERROR("There is no " << i << "th robot");
        return NULL;
    }
    return graspitWorld->getRobot(i);
}


Robot * GraspItSceneManager::getRobot(const unsigned int i)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return NULL;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Graspit world is NULL");
        return NULL;
    }
    return getRobotNoCheck(i);
}




int GraspItSceneManager::moveObject(const std::string& name, const EigenTransform& worldTransform)
{
    if (name.empty())
    {
        PRINTERROR("Cannot move an object without a name");
        return -1;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot move " << name << " with no initialized graspitWorld");
        return -2;
    }

    return moveObjectNoCheck(name, worldTransform);
}

int GraspItSceneManager::moveObjectNoCheck(const std::string& name, const EigenTransform& worldTransform)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    // Check that no object with same name exists
    Body * existingBody = getBodyNoCheck(name);
    if (!existingBody) return -1;
    transf trans = getGraspitTransform(worldTransform);
    existingBody->setTran(trans);
    return 0;
}


bool GraspItSceneManager::removeElement(WorldElement* elem, const bool deleteInstance)
{
    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return false;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot remove element with no initialized graspitWorld");
        return false;
    }
    removeElementNoCheck(elem, deleteInstance);
    return true;
}


void GraspItSceneManager::removeElementNoCheck(WorldElement* elem, const bool deleteInstance)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    // graspitWorld->destroyElement(elem,deleteInstance);
    // PRINTMSG("Now the number of bodies is "<<graspitWorld->getNumBodies());

    // cannot use removeRobot() for the robot because this deletes the object too,
    // and deleteInstance may be false!

    Robot * robot = dynamic_cast<Robot*>(elem);
    if (robot)
    {
        // this is a robot so we also need to remove the link bodies which
        // are added in World::addRobot().
        // unfortunately this is not handled in World::destroyElement (or in
        // World::removeRobot(), so the bodies end up remaining in the scene
        if (robot->getBase())
        {
            graspitWorld->destroyElement(robot->getBase(), false);
        }

        for (int f = 0; f < robot->getNumChains(); f++)
        {
            for (int l = 0; l < robot->getChain(f)->getNumLinks(); l++)
            {
                Body * link = robot->getChain(f)->getLink(l);
                if (link)
                {
                    graspitWorld->destroyElement(link, false);
                    // PRINTMSG("Removed link "<<link->getName().toStdString()<<" from world.");
                }
            }
        }
    }

    graspitWorld->destroyElement(elem, deleteInstance);

    if (!deleteInstance)
    {
        // remove the World object parent and pass on the responsibility to
        // delete this object to fakeQObjectParent.
        // Explanation: The graspit World is normally QObject-parent to
        // all Robots and Objects. Further, all robots and objects are
        // managed from within in the world, so they are properly deleted in the
        // World destructor. However, now we have removed this registration
        // from the world. So instead, the Robot and Body objects will be
        // destroyed from within the QObject destructor, when it destroys
        // all children of it (this happens when World is destroyed, right
        // after its destructor is called).
        // However, the Body and Robot destructors still access to the
        // World and to various other objects which have been destroyed
        // in the World destructor already. Then there is a memory corruption.
        elem->setParent(fakeQObjectParent);
        // setting to NULL would pass reponsibility to delete to caller
        // instead, but only until they may re-add the element to the World.
        // this makes usage of the interface too complicated, so it's better
        // to use fakeQObjectParent and tell users of GraspItSceneManager that
        // they simply never have to worry about destroying objects.

        // elem->setParent(NULL);
    }
}


int GraspItSceneManager::removeObject(const std::string& name)
{
    if (name.empty())
    {
        PRINTERROR("Cannot remove an object without a name");
        return -1;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot remove " << name << " with no initialized graspitWorld");
        return -2;
    }

    return removeObjectNoCheck(name);
}


int GraspItSceneManager::removeObjectNoCheck(const std::string& name)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    // Check that no object with same name exists
    Body * existingBody = getBodyNoCheck(name);
    if (!existingBody) return -1;
    // Options in World to delete objects:
    // * World::destroyElement: removes any element from the scene (robot, object, body...), and from
    //   robotVec/handVec/etc. Allows optionally to delete object pointed to. Also emits signals.
    // * World::removeElementFromSceneGraph just removes the stuff from IV tree
    // * World::removeRobot calls removeElementFromSceneGraph and additionally removes it
    //   from robotVec and handVec AND deletes the object. Does not emit signals.
    removeElementNoCheck(existingBody, true);
    return 0;
}


int GraspItSceneManager::removeRobot(const std::string& name)
{
    if (name.empty())
    {
        PRINTERROR("Cannot remove an object without a name");
        return -1;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot remove " << name << " with no initialized graspitWorld");
        return -2;
    }

    return removeRobotNoCheck(name);
}


int GraspItSceneManager::removeRobotNoCheck(const std::string& name)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    // Check that no object with same name exists
    Robot * existingRobot = getRobotNoCheck(name);
    if (!existingRobot) return -1;
    removeElementNoCheck(existingRobot, true);
    return 0;
}


int GraspItSceneManager::moveRobot(const std::string& name, const EigenTransform& worldTransform)
{
    if (name.empty())
    {
        PRINTERROR("Cannot move a robot without a name");
        return -1;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -2;
    }
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("Cannot move " << name << " with no initialized graspitWorld");
        return -2;
    }

    return moveRobotNoCheck(name, worldTransform);
}


int GraspItSceneManager::moveRobotNoCheck(const std::string& name, const EigenTransform& worldTransform)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    // Check that no object with same name exists
    Robot * existingRobot = getRobotNoCheck(name);
    if (!existingRobot) return -1;
    transf trans = getGraspitTransform(worldTransform);
    existingRobot->setTran(trans);
    return 0;
}



int GraspItSceneManager::setCurrentHand(const std::string& robotName)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    Robot * robot = getRobot(robotName);
    if (!robot)
    {
        PRINTERROR("Robot " << robotName << " could not be found.");
        return -1;
    }

    if (!robot->inherits("Hand"))
    {
        PRINTERROR("Robot " << robotName << " is not of type Hand");
        return -2;
    }

    Hand * hand = dynamic_cast<Hand*>(robot);
    if (!hand)
    {
        PRINTERROR("Could not cast robot to Hand type");
        return -2;
    }
    graspitWorld->setCurrentHand(hand);
    return 0;
}


int GraspItSceneManager::setCurrentGraspableObject(const std::string& objectName)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    Hand * hand = getCurrentHand();
    if (!hand)
    {
        PRINTERROR("No hand currently selected");
        return -1;
    }

    GraspableBody * gObject = getGraspableBody(objectName);
    if (!gObject)
    {
        PRINTERROR("No graspable object " << objectName << " found.");
        return -2;
    }

    hand->getGrasp()->setObjectNoUpdate(gObject);
    return 0;
}

int GraspItSceneManager::setGraspableObject(const std::string& robotName, const std::string& objectName)
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    Robot * robot = getRobot(robotName);
    if (!robot)
    {
        PRINTERROR("Robot " << robotName << " not found.");
        return -1;
    }

    if (!robot->inherits("Hand"))
    {
        PRINTERROR("Robot " << robotName << " is not of type Hand");
        return -2;
    }

    Hand * hand = dynamic_cast<Hand*>(robot);
    if (!hand)
    {
        PRINTERROR("Could not cast robot " << robotName << " to Hand type");
        return -2;
    }

    GraspableBody * gObject = getGraspableBody(objectName);
    if (!gObject)
    {
        PRINTERROR("No graspable object " << objectName << " found.");
        return -3;
    }

    hand->getGrasp()->setObjectNoUpdate(gObject);
    return 0;
}


GraspableBody * GraspItSceneManager::getCurrentGraspableBody()
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    Hand * hand = getCurrentHand();
    if (!hand)
    {
        PRINTERROR("No hand currently selected");
        return NULL;
    }

    return hand->getGrasp()->getObject();
}


const GraspableBody * GraspItSceneManager::readCurrentGraspableBody() const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);

    const Hand * hand = readCurrentHand();
    if (!hand)
    {
        PRINTERROR("No hand currently selected");
        return NULL;
    }

    return hand->getGrasp()->getObject();
}




const Hand * GraspItSceneManager::readCurrentHand() const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (graspitWorld) return graspitWorld->getCurrentHand();
    return NULL;
}



Hand * GraspItSceneManager::getCurrentHand()
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (graspitWorld) return graspitWorld->getCurrentHand();
    return NULL;
}


int GraspItSceneManager::addRobot(Robot* robot, const EigenTransform& worldTransform)
{
    if (!robot)
    {
        PRINTERROR("Trying to add NULL robot");
        return -2;
    }

    std::string robotName = robot->getName().toStdString();

    if (robotName.empty())
    {
        PRINTERROR("Can only add robots with a name");
        return -3;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -1;
    }

    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("World not initialized");
        return -1;
    }

    // double-check that neither an instance with same pointer OR name is loaded
    if (isRobotLoaded(robot) || isRobotLoaded(robotName))
    {
        PRINTERROR("Robot " << robotName << " already exists in world (as name or same pointer).");
        return -4;
    }


    if (robot->getWorld() != graspitWorld)
    {
        std::stringstream s;
        s << "The world registered at the Robot is not the same as the current world. ";
        s << "Such changes to the WorldElement objects has not been considered ";
        s << "in the current implementation. Other things could be not initialized ";
        s << "properly. Cannot add this object to the world at this stage, use only ";
        s << "objects which have been created by the graspit world in the first place.";
        PRINTERROR(s.str());
        return -2;
    }


    // we need to add the robot links to the CollisionInterface again,
    // World::addRobot() does not do this.
    std::vector<DynamicBody *> allLinkVec;
    robot->getAllLinks(allLinkVec);
    std::vector<DynamicBody *>::iterator it;
    for (it = allLinkVec.begin(); it != allLinkVec.end(); ++it)
    {
        DynamicBody * link = *it;
        if (link)
        {
            // PRINTMSG("Adding link '"<<link->getName().toStdString()<<"' to collision environment");
            link->addToIvc();
        }
    }
    // XXX TODO we probably also have to handle Robot::getAllAttachedRobots()!!

    transf trans = getGraspitTransform(worldTransform);
    robot->setTran(trans);

    graspitWorld->addRobot(robot, true);
    // set the QObject parent to the GraspIt world so that it is handled
    // properly in its destructor (if it is loaded in the world, it will
    // automatically be destroyed in the graspit destructor anyway,
    // so it should also have the proper parent relationship)
    robot->setParent(graspitWorld);

    return 0;
}

int GraspItSceneManager::addBody(Body * body, const EigenTransform& worldTransform)
{
    if (!body)
    {
        PRINTERROR("Trying to add NULL body");
        return -2;
    }

    std::string bodyName = body->getName().toStdString();

    if (bodyName.empty())
    {
        PRINTERROR("Can only add bodys with a name");
        return -3;
    }

    if (!isInitialized())
    {
        PRINTERROR("Not initialized");
        return -1;
    }

    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!graspitWorld)
    {
        PRINTERROR("World not initialized");
        return -1;
    }

    // double-check that neither an instance with same pointer OR name is loaded
    if (isObjectLoaded(body) || isObjectLoaded(bodyName))
    {
        PRINTERROR("Body with name " << bodyName << " already exists in world (as name or same pointer).");
        return -4;
    }

    if (body->getWorld() != graspitWorld)
    {
        std::stringstream s;
        s << "The world registered at the Body is not the same as the current world. ";
        s << "Such changes to the WorldElement objects has not been considered ";
        s << "in the current implementation. Other things could be not initialized ";
        s << "properly. Cannot add this object to the world at this stage, use only ";
        s << "objects which have been created by the graspit world in the first place.";
        PRINTERROR(s.str());
        return -2;
    }

    transf trans = getGraspitTransform(worldTransform);
    body->setTran(trans);

    // we need to add the body to the CollisionInterface again,
    // World::addBody() does not do this.
    body->addToIvc();
    graspitWorld->addBody(body);

    // set the QObject parent to the GraspIt world so that it is handled
    // properly in its destructor (if it is loaded in the world, it will
    // automatically be destroyed in the graspit destructor anyway,
    // so it should also have the proper parent relationship)
    body->setParent(graspitWorld);

    return 0;
}





unsigned int GraspItSceneManager::getNumGraspableBodies() const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    return  graspitWorld->getNumGB();
}

unsigned int GraspItSceneManager::getNumBodies() const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    return graspitWorld->getNumBodies();
}

unsigned int GraspItSceneManager::getNumRobots() const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    return graspitWorld->getNumRobots();
}





bool GraspItSceneManager::isRobotLoaded(const std::string& name) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    const Robot * r = readRobot(name);
    return r != NULL;
}


bool GraspItSceneManager::isObjectLoaded(const std::string& name) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    const Body * b = readBody(name);
    return b != NULL;
}

bool GraspItSceneManager::isRobotLoaded(const Robot * r) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!r)
    {
        PRINTERROR("Cannot check for NULL robot");
        return false;
    }

    int numR =  graspitWorld->getNumRobots();
    for (int i = 0; i < numR; ++i)
    {
        const Robot * rw = graspitWorld->getRobot(i);
        if (rw == r)
        {
            return true;
        }
    }
    return false;
}


bool GraspItSceneManager::isObjectLoaded(const Body * b) const
{
    UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    if (!b)
    {
        PRINTERROR("Cannot check for NULL object");
        return false;
    }
    int nb =  graspitWorld->getNumBodies();
    for (int i = 0; i < nb; ++i)
    {
        const Body * rb = graspitWorld->getBody(i);
        if (rb == b)
        {
            return true;
        }
    }
    return false;
}

