#ifndef GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGER_H
#define GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGER_H

#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Scene Manager for the GraspIt world.

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
#endif

#include <string>
#include <map>

#include <QObject>

#include <grasp_planning_graspit/ThreadImpl.h>
#include <grasp_planning_graspit/SharedPtr.h>
#include <grasp_planning_graspit/GraspItTypes.h>


class GraspableBody;
class Hand;
class EGPlanner;
class World;
class GraspitCore;
class SoSensor;
class transf;
class Robot;
class Body;
class GraspableBody;
class WorldElement;


namespace GraspIt
{

class GraspItAccessor;

/**
 * \brief Provides a scene manager which can be used to load and access the GraspIt! world.
 *
 * This class manages access to the graspit world object and operations which can be performed on it.
 * It also runs an event loop, which may be implemented in various ways in subclasses. In some
 * implementations, this thread may have a special meaning, e.g. when it is also the thread which runs
 * a SoQt main loop. This thread may be required to do operations use elsewhere.
 * Instances of class GraspItAccessor may get access to the thread which runs this event loop
 * via GraspItAccessor::addAsIdleListener().
 *
 * There should only be once instance of GraspItSceneManager for each GraspIt! world. Access to this world and
 * running of other GraspIt algorithms should be handled by GraspItAccessor instances.
 * There can be several GraspItAccessor objects operating on the same graspit world.
 *
 * GraspItSceneManager and GraspItAccessor work closely together as friends, therefore some of the protected
 * methods in this class have also been made available for GraspItAccessor subclasses.
 *
 * GraspItSceneManager can be extended in various ways by subclasses which access individual parts of
 * the original GraspIt! code.
 * However it is recommended to have a look at the class GraspItAccessor to see if access to the GraspIt!
 * world is enough for the implementation. To keep things modular, this class should only be re-implemented in
 * subclasses if access via GraspItAccessor is not sufficient.
 * GraspItSceneManager basically only should handle the main scene manager loop and manage the world,
 * similar to how GraspitCore in the original GraspIt! source does.
 *
 * Before a GraspItSceneManager instance can be used, it has to be initialized by calling initialize().
 * It is expected that subclasses constructors call this method, but there is no harm in calling it several times.
 * Methods waitUntilReady() and isReady() can be used to check whether the object has finished initializing, but
 * usually this should have been made sure by initialize(), so it should not be required.
 * Similarly, before destruction, the method shutdown() has to be called. It is expected that subclasses call it
 * from their destructors, but there's no harm in calling it several times.
 *
 * Worlds, robots and objects/obstacles can only be loaded from files. This is because from within the XML files, other
 * XML files are referenced (e.g. contact files, robot/object files, etc.). Moreover, the current graspit source is laid
 * out for loading from files (or specific database access only). The current source only allows bodies (objects/obstacles)
 * to be specified as XML within a world file (without referencing another file).
 * Loading robots and contact files still requires actual files to be present. Threrefore, at the current stage, rather
 * than providing *load-from-xmlstring* methods, it is easier to create a temporary folder with all XML files and then
 * call the load* methods of this class with the filenames.
 * This can be done in an external helper class if required.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GraspItSceneManager
{
    friend class GraspItAccessor;
public:
    GraspItSceneManager();
    virtual ~GraspItSceneManager();

    /**
     * Initializes the scene manager incl. starting the event loop
     * and creating the graspit world.
     *
     * This should be called ideally from the
     * subclasses constructors. It is necessary to call it from the subclasses
     * constructors, instead of from the base class destructor, otherwise it causes
     * calls to pure virutal methods.
     *
     * It is always safe for the user to call this function in any case as a
     * precaution. There is no harm in calling it several times.
     * *Info:* This method calls the protected virtual initializeCore().
     */
    void initialize();

    /**
     * Shuts down the scene manager (basically undoes initialize()).
     * This includes destroying the graspit world, exiting the scene manager
     * event loop.
     * All GraspItAccessor classes which registered with this GraspItSceneManager
     * instance are removed, and their GraspItAccessor::onSceneManagerShutdown() is called.
     *
     * This method should be called before destroying this instance, ideally from the
     * subclasses destructors. It is necessary to call it from the subclasses
     * destructors, instead of from the base class destructor, otherwise it causes
     * calls to pure virutal methods.
     *
     * It is always safe for the user to call this function in any case as a
     * precaution. There is no harm in calling it several times.
     * *Info:* This method calls the protected virtual destroyCore().
     */
    void shutdown();

    /**
     * Returns true if initialize() has been called (and not shutdown() after),
     * and isReady() also returns true.
     */
    bool isInitialized() const;

    /**
     * Returns true if interface has finished initialziing (which may happen in
     * a separate thread initiated in constructor) and the interface is ready to use
     */
    virtual bool isReady() const = 0;

    /**
     * Waits until isReady() returns true
     */
    virtual void waitUntilReady() const = 0;

    std::vector<std::string> getObjectNames(bool graspable) const;
    std::vector<std::string> getRobotNames() const;


    /**
     * Loads graspitWorld from an XML file.
     * \retval 0 success
     * \retval -1 failed to load world with graspit source
     * \retval -2 interface not initialized. Call waitUntilReady() before you load a world.
     * \retval -3 file does not exist
     */
    int loadWorld(const std::string& filename);


    /**
     * Loads robot (hand) from XML file and adds it to the graspit world.
     *
     * If it is the ONLY robot loaded so far, and the robot is of a "Hand" type, it also sets this hand
     * as the currently loaded hand. This is like calling setCurrentHand(robotName).
     * In this case, the first graspable object loaded in the world will be defined by default as the object
     * to grasp by this robot (see also graspit source for World::addRobot()).
     * You can change this with setGraspableObject(const std::string&, const std::string&).
     *
     * The transform into the world can be specified but this is optional.
     * For the actual planning, placing the robot at the origin of the scene with
     * default orientation is ok, even if it initially collides with the object.
     * \param filename must contain the full path to the robots XML file.
     * \param worldTransform the transform to apply to the world
     * \param robotName the unique name to give this particular robot.
     * \see World::importRobot(QString filename)
     * \retval 0 success
     * \retval -1 failed to load world with graspit source
     * \retval -2 interface not initialized. Call waitUntilReady() before you load a robot.
     * \retval -3 file does not exist
     * \retval -4 a robot with the same name already is loaded in the world.
     * \retval -5 name is empty. You have to load a robot with a unique name.
     */
    int loadRobot(const std::string& filename, const std::string& robotName,
                  const EigenTransform& worldTransform = EigenTransform::Identity());

    /**
     * Removes the robot from the world and destroys the object which was loaded.
     * \retval 0 success
     * \retval -1 no such robot exists
     * \retval -2 interface not initialized. Call waitUntilReady() before you load an object.
     */
    int removeRobot(const std::string& robotName);

    /**
     * Moves the robot in the scene.
     * \retval 0 success
     * \retval -1 no such robot exists
     * \retval -2 interface not initialized. Call waitUntilReady() before you load an object.
     */
    int moveRobot(const std::string& robotName, const EigenTransform& worldTransform);


    /**
     * Loads a graspable object from XML,IV,OFF or PLY file and adds it to the grapsit world.
     *
     * Objects can be loaded as graspable bodies or obstacles. The only difference is that graspable
     * bodies can be grasped. Names must be unique among all graspable bodies and obstacles.
     *
     * If this object is to be loaded as graspable, and there are no graspable bodies in the world yet,
     * ALL hands of robots of type "Hand" will be set to grasp this object by default. This is like
     * calling setGraspableObject(robot,name). This makes sense because if there are no objects
     * loaded (e.g. even if they have been removed with removeObject(const std::string&)), NO
     * hand will point to a current object to grasp. This way, all hands are initialized to grasp
     * something, as soon as an object exists. You can change this for each  hand with
     * setGraspableBody(const std::string& const std::string&).
     *
     * The transform into the world can be specified but this is optional unless a specific position
     * relative to an obstacle is required. For the actual planning, placing only object and robot at the origin
     * of the scene with default orientation is ok.
     * \see World::importBody(QString bodyType,QString filename)
     * \param asGraspable add the object as graspable object. If false, it will be an obstacle.
     * \param name the unique name to give this particular object.
     * \retval 0 success
     * \retval -1 failed to load world with graspit source
     * \retval -2 interface not initialized. Call waitUntilReady() before you load an object.
     * \retval -3 file does not exist
     * \retval -4 an object with the same name already is loaded in the world.
     * \retval -5 name is empty. You have to load an object with a unique name.
     */
    int loadObject(const std::string& filename, const std::string& name, const bool asGraspable = true,
                   const EigenTransform& worldTransform = EigenTransform::Identity());

    /**
     * Removes the object from the world and destroys the object which was loaded.
     * \retval 0 success
     * \retval -1 no such object exists
     * \retval -2 interface not initialized. Call waitUntilReady() before you load an object.
     */
    int removeObject(const std::string& name);

    /**
     * Moves the object (a graspable object or an obstacle) in the scene.
     * \retval 0 success
     * \retval -1 no such object exists
     * \retval -2 interface not initialized. Call waitUntilReady() before you load an object.
     */
    int moveObject(const std::string& name, const EigenTransform& worldTransform);

    /**
     * Sets the current Hand. This is like selecting it. Following algorithms will
     * refer to this hand to be used.
     * You may still have to specify the object to grasp by this hand, if this was not
     * done before for this hand. You can do this with setCurrentGraspableObject().
     *
     * \retval 0 success
     * \retval -1 robot could not be found. This could also happen if initialize() wasn't called.
     * \retval -2 robot is not of type "Hand"
     */
    int setCurrentHand(const std::string& robotName);

    /**
     * Sets the GraspableObject for the current hand (set by setCurrentHand()).
     * Following algorithms will refer to this graspable object to be used.
     * \retval 0 success
     * \retval -1 no hand currently set as default. Do this with setCurrentHand(const std::string&).
     * \retval -2 this object was not found. This could also happen if initialize() wasn't called.
     *
     */
    int setCurrentGraspableObject(const std::string& objectName);

    /**
     * Set the object to grasp for this robot. This does not affect currently loaded hand
     * and graspable object which can be set with setCurrentHand(const std::string&) and
     * setCurrentGraspableObject(const std::string&).
     * \retval 0 success
     * \retval -1 could not find this robot. This could also be because initialize() wasn't called.
     * \retval -2 robot is not of type "Hand"
     * \retval -3 this object was not found. This could also happen if initialize() wasn't called.
     */
    int setGraspableObject(const std::string& robotName, const std::string& objectName);

    /**
     * Save the currently loaded world as GraspIt world file.
     * \param createDir if true, the directory in which the file is to be saved is created if it does not exist.
     */
    bool saveGraspItWorld(const std::string& filename, bool createDir = false);

    /**
     * Save the currently loaded world as Inventor file.
     * \param createDir if true, the directory in which the file is to be saved is created if it does not exist.
     */
    bool saveInventorWorld(const std::string& filename, bool createDir = false);

    /**
     * Saves the robot as inventor file, if it was loaded in the world before.
     */
    bool saveRobotAsInventor(const std::string& filename, const std::string& robotName,
                                   const bool createDir=false, const bool forceWrite=false);

    /**
     * Saves the object as inventor file, if it was loaded in the world before.
     */
    bool saveObjectAsInventor(const std::string& filename, const std::string& name,
                              const bool createDir=false, const bool forceWrite=false);

    /**
     * returns true if a robot with this name is currently loaded in the world
     */
    bool isRobotLoaded(const std::string& name) const;
    /**
     * returns true if an object with this name is currently loaded in the world
     */
    bool isObjectLoaded(const std::string& name) const;

protected:
    /**
     * This method is supposed to initialize the Core instance (field core).
     * If any other threads are started by this function which
     * need to complete initialization routines, this method has to block until all initialization
     * is finished and the Core instance is fully initialized.
     * This method is expected to also create a scene manager event loop thread which regularly
     * should call processIdleEvent() (each time after scheduleIdleEvent() has been called).
     */
    virtual void initializeCore() = 0;

    /**
     * Stops the scene manager thread, the event loop,
     *  and everything which needs to be destroyed/shut down
     * (basically reverting what was initialized in initializeCore()).
     * Also is expected to destroy the core object and sets it to NULL.
     */
    virtual void destroyCore() = 0;

    /**
     * Creates a new graspit world object. In case the world needs special initializing, the
     * implementation is left to the subclasses.
     * \return pointer to the new world object which is now maintained by GraspitCore.
     */
    virtual World * createNewGraspitWorld() = 0;

    /**
     * Subclasses which support Qt and which use the
     * same thread which runs the SoQt loop for the event loop
     * as well, should return true here. This can be used
     * by GraspItAccessor classes to ensure that Qt is supported
     * in the event thread.
     */
    virtual bool eventThreadRunsQt() const
    {
        return false;
    }



    /**
     * Should be called from subclasses scene manager thread each time after an idle event has been
     * scheduled with scheduleIdleEvent().
     */
    void processIdleEvent();


    /**
     * Compares if a robot with the same pointer address is loaded.
     */
    bool isRobotLoaded(const Robot * robot) const;
    /**
     * Compares if an object with the same pointer address is loaded.
     */
    bool isObjectLoaded(const Body * body) const;

    /**
     * Returns the number of graspable bodies loaded.
     */
    unsigned int getNumGraspableBodies() const;

    /**
     * Returns the number of robots loaded.
     */
    unsigned int getNumRobots() const;

    /**
     * Returns the number of bodies loaded. Includes graspable and non-graspable objects
     * and parts of robots (e.g. links in the chain).
     * this is only useful to check before calling getBody(unsigned int) to
     * check for the index.
     */
    unsigned int getNumBodies() const;

    /**
     * Tries to get a lock on the graspit world.
     * While the world is locked, all methods which access it will block.
     */
    bool tryLockWorld()
    {
        return graspitWorldMtx.try_lock();
    }

    /**
     * Locks the graspit world.
     * While the world is locked, all methods which access it will block.
     */
    void lockWorld()
    {
        graspitWorldMtx.lock();
    }

    /**
     * Unlocks the graspit world.
     * While the world is locked, all methods which access it will block.
     */
    void unlockWorld()
    {
        graspitWorldMtx.unlock();
    }

    /**
     * Returns a unique lock for the world which can be used within scopes
     * for convenience.
     * While the world is locked, all methods which access it will block.
     */
    UNIQUE_RECURSIVE_LOCK getUniqueWorldLock()
    {
        return UNIQUE_RECURSIVE_LOCK(graspitWorldMtx);
    }

    /**
     * Returns the hand currently loaded as main hand in the world.
     * This method has to be handled with care, as it can break thread safety. The current hand is
     * part of the current world, and this pointer is returned. If the world changes, this pointer
     * is not valid any more.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     */
    Hand * getCurrentHand();

    /**
     * Returns the hand currently loaded as main hand in the world.
     * This method has to be handled with care, as it can break thread safety. The current hand is
     * part of the current world, and this pointer is returned. If the world changes, this pointer
     * is not valid any more.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     */
    const Hand * readCurrentHand() const;

    /**
    * Returns the object currently loaded as the object to grasp for the currently loaded hand.
    * This method has to be handled with care, as it can break thread safety. The current object is
    * part of the current world, and this pointer is returned. If the world changes, this pointer
    * is not valid any more.
    * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
    * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
    * However keep in mind that other threads trying to access the world will be unable to do so
    * while the lock is held.
    * \return NULL on error (e.g. no hand could be retrieved with getCurrentHand()) or if
    * no object was set as current for this hand.
    */
    GraspableBody * getCurrentGraspableBody();

    /**
     * Returns the object currently loaded as the object to grasp.
     * This method has to be handled with care, as it can break thread safety. The current object is
     * part of the current world, and this pointer is returned. If the world changes, this pointer
     * is not valid any more.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     */
    const GraspableBody * readCurrentGraspableBody() const;



    /**
     * Returns the robot with this name in the currently loaded world.
     * This method has to be handled with care, as it can break thread safety. The robot is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     * \param name name of the robot to get.
     * \return NULL if robots cannot be retrieved at this stage, or there is no such robot. Otherwise the pointer
     */
    Robot * getRobot(const std::string& name);

    const Robot * readRobot(const std::string& name) const;

    /**
     * Gets the i'th robot loaded in the world.
     * This method has to be handled with care, as it can break thread safety. The robot is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     * \return the i'th graspable body in the world. 0 is the first object. NULL if no objects
     * can be retrieved at this stage or there is no such object.
     */
    Robot * getRobot(const unsigned int i);



    /**
     * Returns the graspable body with this name in the currently loaded world.
     * This method has to be handled with care, as it can break thread safety. The object is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     * \param name name of the object to get.
     * \return NULL if objects cannot be retrieved at this stage, or there is no such object. Otherwise the pointer
     */
    GraspableBody * getGraspableBody(const std::string& name);

    const GraspableBody * readGraspableBody(const std::string& name) const;

    /**
     * Returns the i'th graspable body loaded in the world.
     * This method has to be handled with care, as it can break thread safety. The object is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     * \return the i'th graspable body in the world. 0 is the first object. NULL if no objects
     * can be retrieved at this stage or there is no such object.
     */
    GraspableBody * getGraspableBody(const unsigned int i);


    /**
     * Returns the body (can be either an obstacle or a graspable body)  with this name in the
     * currently loaded world.
     * This method has to be handled with care, as it can break thread safety. The object is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     * \param name name of the object to get.
     * \return NULL if there is no such object. Otherwise the pointer
     */
    Body * getBody(const std::string& name);

    const Body * readBody(const std::string& name) const;

    /**
     * Returns the i'th body (can be either graspable or obstacle, or a part of a robot) loaded in the scene.
     * This method has to be handled with care, as it can break thread safety. The object is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     */
    Body * getBody(const unsigned int i);

    /**
     * Removes a specific world element (graspit Robot or Body).
     * This method has to be handled with care, as it can break thread safety. The object is
     * part of the current world, and this pointer is returned. If the world is changed while operations
     * on this pointer are made, this pointer may get invalidated.
     * To lock the world, subclasses or friend classes can use tryLockWorld(), lockWorld() and
     * unlockWorld(). To avoid concurrency issues, this should be used before this method is called.
     * However keep in mind that other threads trying to access the world will be unable to do so
     * while the lock is held.
     *
     * IMPORTANT: You can choose to not destroy the element associated
     * with the WorldElement by setting deleteInstance parameter to true.
     * However, GraspItSceneManager will still internally delete this instance from within
     * the destructor. So never delete an element removed, even if deleteInstance parameter
     * was set to false. For more information, see comment for fakeQObjectParent.
     *
     * \param deleteInstance whether to delete the instance (parameter elem) as well.
     */
    bool removeElement(WorldElement* elem, const bool deleteInstance);

    /**
     * Adds the robot to the world (see also World::addRobot()). With this,
     * you can for example re-add a robot which was previously removed
     * with removeElement(WorldElement*,const bool) and deleteInstance parameter
     * set to false.
     *
     * \retval 0 success
     * \retval -1 Interface not initialized. Call waitUntilReady() before you add a robot.
     * \retval -2 Robot object is invalid (e.g. not initialized properly) and cannot
     * be used.
     * \retval -3 Empty name: robot must have a name in order to be added.
     * \retval -4 A robot with the same name already is loaded in the world.
     */
    int addRobot(Robot* robot, const EigenTransform& worldTransform);

    /**
     * Adds the body to the world (see also World::addBody()). With this,
     * you can for example re-add a body which was previously removed
     * with removeElement(WorldElement*,const bool) and deleteInstance parameter
     * set to false.
     *
     * \retval 0 success
     * \retval -1 Interface not initialized. Call waitUntilReady() before you add a robot.
     * \retval -2 Body object is invalid (e.g. not initialized properly) and cannot
     * be used.
     * \retval -3 Empty name: robot must have a name in order to be added.
     * \retval -4 A body with the same name already is loaded in the world.
     */
    int addBody(Body* body, const EigenTransform& worldTransform);

    /**
     * Schedules a new "Idle Event" which will be called in the next iteration of the scene manager thread.
     *
     * This method is protected so that only subclasses and friend classes (specifically GraspItAccessor) have access to it.
     * GraspItAccessor::scheduleUpdateFromEventLoop() provides the main method for subclasses
     * to use for calling this function.
     *
     * \retval false scene manager thread is not initialized yet.
     */
    virtual bool scheduleIdleEvent() = 0;


private:
    GraspItSceneManager(const GraspItSceneManager& o) {}

    /**
     * Like getGraspableBody(const std::string&) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    GraspableBody * getGraspableBodyNoCheck(const std::string& name);

    const GraspableBody * readGraspableBodyNoCheck(const std::string& name) const;

    /**
     * Like getGraspableBody(const unsigned int) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    GraspableBody * getGraspableBodyNoCheck(const unsigned int i);

    /**
     * Like getBody(const unsigned int) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    Body * getBodyNoCheck(const unsigned int i);

    /**
     * Like getBody(const std::string&) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    Body * getBodyNoCheck(const std::string& name);

    const Body * readBodyNoCheck(const std::string& name) const;

    /**
     * Like getRobot(const unsigned int) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    Robot * getRobotNoCheck(const unsigned int i);

    /**
     * Like getRobot(const std::string&) but does not perform all safety checks, e.g.
     * if world is initialized.
     */
    Robot * getRobotNoCheck(const std::string& name);

    const Robot * readRobotNoCheck(const std::string& name) const;


    /**
     * like moveObject(const std::string&, const EigenTransform&) but does not perform all
     * safety checks, e.g. if world is initialized.
     */
    int moveObjectNoCheck(const std::string& name, const EigenTransform& worldTransform);

    /**
     * like moveRobot(const std::string&, const EigenTransform&) but does not perform all
     * safety checks, e.g. if world is initialized.
     */
    int moveRobotNoCheck(const std::string& name, const EigenTransform& worldTransform);


    /**
     * like removeObject(const std::string&) but does not perform all
     * safety checks, e.g. if world is initialized.
     * If you want to keep the actual Body object, see removeElement(WorldElement*,const bool).
     */
    int removeObjectNoCheck(const std::string& name);

    /**
     * like removeRobot(const std::string&) but does not perform all
     * safety checks, e.g. if world is initialized.
     * If you want to keep the actual Robot object, see removeElement(WorldElement*,const bool).
     */
    int removeRobotNoCheck(const std::string& name);


    /**
     * like removeElement(WorldElement*, const bool) but does not perform all
     * safety checks, e.g. if world is initialized.
     */
    void removeElementNoCheck(WorldElement* elem, const bool deleteInstance);


    /**
     * Registers a GraspItAccessor class to subscribe to updates from within the scene manager event loop.
     * The "idle event" happens at each iteration of the scene manager event loop.
     * This loop is run by the scene manager thread.
     *
     * While Qt is being used, the scene manager thread is also the one which runs the Inventor/Qt stuff.
     * It may be important for instances of type GraspItAccessor to get access
     * to this thread, so that selected Qt objects can be created, signals/slots connected, etc.
     *
     * The method GraspItAccessor::idleEventFromSceneManager() will be called from the scene manager thread
     * IF the method GraspItAccessor::isScheduledForUpdate() returns true. This happens every time after
     * scheduleIdleUpdate() has been called.
     *
     * It would be nice to use shared pointers here to avoid the object being notified after it has been
     * destroyed. However, currently used shared pointer implementation(s) cannot be maintained if this method
     * is called from GraspItAccessor itself. Therefore, the GraspItAccessor::shutdown() makes sure to
     * call removeIdleListener() in its destructor to avoid it being used here after its destruction.
     *
     * This method is private so that only friend classes (specifically GraspItAccessor) have access to it.
     * GraspItAccessor::addAsIdleListener() provides the main method for subclasses to use for calling this function.
     */
    bool addIdleListener(GraspItAccessor* s);

    /**
     * Goes through the list of GraspItAccessors which were added with addIdleListener(GraspItAccessor*)
     * and removes the one with the same name.
     *
     * This method is private so that only friend classes (specifically GraspItAccessor) have access to it.
     * GraspItAccessor::removeFromIdleListeners() provides the main method for subclasses to use for
     * calling this function.
     *
     * \return false if the GraspItAccessor object was not registered, otherwise true.
     */
    bool removeIdleListener(GraspItAccessor* s);


protected:
    // internally needed object for the inventor stuff.
    GraspitCore * core;

    //! Points to the main graspitWorld which is maintained by core.
    // This is just a convenience pointer.
    World * graspitWorld;
    mutable RECURSIVE_MUTEX graspitWorldMtx;

private:
    std::map<std::string, GraspItAccessor*> registeredAccessors;
    MUTEX registeredAccessorsMtx;


    // Initialization flag set to true after initialize() was called.
    bool initialized;

    /**
     * QObject which becomes new parents for WorldElements which have
     * been removed from the world, but not destroyed
     * (see removeElement(WorldElement*,const bool)).
     *
     * Each WorldElement NEEDS to be deleted either a) from within the
     * World destructor or b) before World is deleted.
     * Explanation: The graspit World is normally QObject-parent to
     * all WorldElement objects. Further, all robots and objects are
     * registered in the World object, so they are properly deleted in the
     * World destructor before the World is completely destroyed.
     * However, the method removeElement() removes this registration
     * from the world. This means that instead, the Robot and Body
     * objects will automatically be destroyed from within their
     * QObject-parent QObject::~QObject(), which destroys
     * all children of it (this happens right AFTER the World destructor
     * is called; then the superclass destructor is called).
     * However, the Body and Robot destructors still access the
     * World object and to various other objects which have been destroyed
     * in the World destructor already. Then, there is a memory corruption.
     * So for all elements which have NOT been destroyed BUT removed from
     * the World, we need a temporary QObject parent which will take
     * care of deleting all these objects BEFORE the World object
     * is destroyed.
     */
    QObject *fakeQObjectParent;
};
}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGER_H
