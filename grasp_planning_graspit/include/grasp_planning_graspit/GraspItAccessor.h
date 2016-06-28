#ifndef GRASP_PLANNING_GRASPIT_GRASPITACCESSOR_H
#define GRASP_PLANNING_GRASPIT_GRASPITACCESSOR_H

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


// #include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/SharedPtr.h>
#include <grasp_planning_graspit/GraspItSceneManager.h>
#include <grasp_planning_graspit/GraspItTypes.h>
#include <string>

namespace GraspIt
{

class GraspItSceneManager;

/**
 * \brief Interface which can access the %GraspIt world and run algorithms on it.
 *
 * This interface can be implemented in various ways by algorithms which want to access the GraspIt world.
 * The graspit world is managed by the GraspItSceneManager instance which objects of this class have a
 * reference to.
 *
 * About the relationship between GraspItSceneManager and GraspItAccessor:
 * - While there should only be one instance of a %GraspItSceneManager, there can be several %GraspItAccessor
 *     instances operating on the same %GraspItSceneManager and accessing the world.
 * - %GraspItAccessor subclasses have access to the %GraspItSceneManager instance via a getGraspItSceneManager().
 *    With this, they can access all its public functions.
 * - Both classes are abstract base classes which cooperate as friend classes. Selected protected methods of
 *     %GraspItSceneManager have been made accessible to subclasses of %GraspItAccessor
 *     via some of the protected functions in this class.
 * - The reference to %GraspItSceneManager is kept locally as a shared pointer within this class.
 *     If shared pointers are used properly, this ensures that %GraspItAccessor
 *     is always deleted *before* %GraspItSceneManager. So make sure that you keep another shared pointer of
 *     the %GraspItSceneManager elsewhere to ensure correct delete order.
 * - Subclasses can register themselves for updates by the main scene manager loop which is run in %GraspItSceneManager.
 *      The scene manager loop is run by its own thread.
 *      To register the callback function idleEventFromSceneManager() to be called from the scene manager thread,
 *      the method addAsIdleListener() can be used. Please read more about this in the method documentation.
 *
 * Subclasses which want to make use of QT signals and slots, or any other mechanisms proivded by QObject and the Qt metaobject system,
 * will have to additionally derive from QObject, and the MOC files have to be generated for it. This will only work for
 * GraspItSceneManager implementations where the event loop is run by the same thread which also runs the SoQt main loop.
 * You can check this with eventThreadRunsQt().
 *
 * *Important Note:*
 *
 * If addAsIdleListener() is ever called, the subclasses *themselves* **must** call the method
 * removeFromIdleListeners() from within their own destructors in order to ensure any remaining registration is removed.
 * Otherwise, GraspItSceneManager (which is to be destroyed *after* all GraspItAccessor instances) may still try to call
 * idleEventFromSceneManager() of an object which is still registered but has already been destroyed.
 *
 * *Question*: Why can removeFromIdleListeners() not be called from within this base class destructor, and must
 * be called from within the subclasses' destructors?
 * *Answer*: The GraspItAccessor subclass destructor is called  *before* the GraspItAccessor destructor. So if the
 * GraspItAccessor superclass itself would take care of calling removeFromIdleListeners(),
 * access could still be happening on the already deleted subclass, which can lead to segfaults or calls of pure virtual methods.
 *
 *
 * \author Jennifer Buehler
 * \date December 2015
 */
class GraspItAccessor
{
    friend class GraspItSceneManager;
public:
    /**
     * \param interface the GraspIt interface. CAREFUL! Because a shared pointer is
     * maintained from within this class, make sure you keep at least one shared pointer to the
     * interface somewhere else, so the interface is not destroyed from within the destructor of
     * this class.
     */
    GraspItAccessor(const std::string& objectName, const SHARED_PTR<GraspItSceneManager>& interface);
    virtual ~GraspItAccessor();

    std::string getName() const;

protected:
    /**
     * This method will be called from within the scene manager thread in its next
     * loop, **if** the method addAsIdleListener() was called for this instance **and**
     * isScheduledForIdleEvent() returns true.
     *
     * To trigger repeated calls of this function in each loop of the scene manager
     * event loop, call scheduleForIdleEventUpdate() from within this function.
     *
     * IMPORTANT: To not slow down the whole system this method should be kept very efficient!
     * Any more complex operations should be done in other threads.
     *
     * TIPP: While Qt is still used in GraspItSceneManager, this method will be called
     * from the thread which also runs the Inventor stuff. So it is also possible to use
     * this method only once (without re-scheduling with scheduleForIdleEventUpdate())
     * to create a SoIdleSensor object, which will then also be run from within the
     * inventor thread.
     */
    virtual void idleEventFromSceneManager() = 0;

    /**
     * If this GraspItAccessor is registered with GraspItSceneManager (by a call of
     * addAsIdleListener()), this method will be called from GraspItSceneManager when
     * it shuts down.
     */
    virtual void onSceneManagerShutdown() = 0;


    /**
     * Subscribes this instance to updates from within the scene manager event loop.
     * The loop is run by the scene manager thread.
     * The "idle event" happens at regular intervals, so it's happening repeatedly.
     * This is the event which this method subscribes to.
     *
     * Once subscribed, the method GraspItAccessor::idleEventFromSceneManager() will be called
     * from the scene manager thread *if* the method GraspItAccessor::isScheduledForIdleEvent() returns
     * true. This will be the case each time after GraspItAccessor::scheduleForIdleEventUpdate()
     * is called.
     *
     * **Hint:** While Qt is being used, it may be important for instances of type GraspItAccessor
     * to get access to the scene manager thread, which is also the thread which runs the
     * main SoQt loop. This thread must be used to create selected Qt objects, connect
     * signals/slots, etc. Registering the GraspItAccessor instance as "idle listener" is
     * one possiblity to achieve this. Use the method eventThreadRunsQt() to check whether
     * the event thread also is running Qt.
     */
    bool addAsIdleListener();

    /**
     * Removes this object from the listeners again (undoing addAsIdleListener()).
     */
    bool removeFromIdleListeners();

    /**
     * Commands the thread which runs the scene manager to call GraspItAccessor::idleEventFromSceneManager().
     * This will happen in the next iteration of a loop which the thread runs internally.
     *
     * This method can be called from GraspItAccessor::idleEventFromSceneManager() to trigger a repeated
     * call.
     */
    void scheduleForIdleEventUpdate();

    /**
     * Marks this instance as "not scheduled" for an update through idleEventFromSceneManager().
     */
    void unschedule();

    /**
     * Checks if this instance is "scheduled" for an update through idleEventFromSceneManager().
     */
    bool isScheduledForIdleEvent() const;

    /**
     * Subclasses can get a pointer to the GraspItSceneManager object in order
     * to access public methods.
     * GraspItSceneManager is supposed to be threadsafe, though at beta stage,
     * no guarantees are given yet.
     */
    const SHARED_PTR<GraspItSceneManager>& getGraspItSceneManager();

    const SHARED_PTR<const GraspItSceneManager> readGraspItSceneManager() const;

    /**
     * Calls protected GraspItSceneManager::eventThreadRunsQt()
     */
    bool eventThreadRunsQt() const;

    /**
     * Calls protected GraspItSceneManager::tryLockWorld()
     */
    bool tryLockWorld();

    /**
     * Calls protected GraspItSceneManager::lockWorld()
     */
    void lockWorld();

    /**
     * Calls protected GraspItSceneManager::unlockWorld()
     */
    void unlockWorld();

    /**
     * Calls protected GraspItSceneManager::getUniqueWorldLock()
     */
    UNIQUE_RECURSIVE_LOCK getUniqueWorldLock();

    /**
     * Calls protected GraspItSceneManager::getCurrentHand()
     */
    Hand * getCurrentHand();

    /**
     * Calls  protected GraspItSceneManager::readCurrentHand()
     */
    const Hand * readCurrentHand() const;

    /**
     * Calls  protected GraspItSceneManager::getCurrentGraspableBody()
     */
    GraspableBody * getCurrentGraspableBody();

    /**
     * Calls  protected GraspItSceneManager::readCurrentGraspableBody()
     */
    const GraspableBody * readCurrentGraspableBody() const;

    /**
     * Calls  protected GraspItSceneManager::getRobot(const std::string&)
     */
    Robot * getRobot(const std::string& name);

    /**
     * Calls  protected GraspItSceneManager::getRobot(const unsigned int)
     */
    Robot * getRobot(const unsigned int i);

    /**
     * Calls  protected GraspItSceneManager::getGraspableBody(const std::string&)
     */
    GraspableBody * getGraspableBody(const std::string& name);

    /**
     * Calls  protected GraspItSceneManager::getGraspableBody(const unsigned int)
     */
    GraspableBody * getGraspableBody(const unsigned int i);

    /**
     * Calls  protected GraspItSceneManager::getBody(const std::string&)
     */
    Body * getBody(const std::string& name);

    /**
     * Calls  protected GraspItSceneManager::getBody(const unsigned int)
     */
    Body * getBody(const unsigned int i);

    /**
     * Calls  protected GraspItSceneManager::isRobotLoaded(const Robot*)
     */
    bool isRobotLoaded(const Robot * robot) const;

    /**
     * Calls  protected GraspItSceneManager::isObjectLoaded(const Body*)
     */
    bool isObjectLoaded(const Body * object) const;

    /**
     * Calls protected GraspItSceneManager::removeElement(WorldElement*,const bool).
     * VERY IMPORTANT: Don't delete the element which is removed
     * even if you set deleteInstance to false. Deletion of
     * removed elements is still handled from within
     * GraspItSceneManager.
     */
    bool removeElement(WorldElement* elem, const bool deleteInstance);

    /**
     * Calls  protected GraspItSceneManager::addRobot(Robot*, const EigenTransform&)
     */
    int addRobot(Robot* robot, const EigenTransform& worldTransform);

    /**
     * Calls  protected GraspItSceneManager::addBody(Body*, const EigenTransform&)
     */
    int addBody(Body* body, const EigenTransform& worldTransform);

private:
    /**
     * should never be used to stay safe. Subclasses should declare their copy constructor private.
     */
    GraspItAccessor(const GraspItAccessor& o) {}

    /**
     * should never be used to stay safe. Subclasses should declare their default constructor private.
     */
    GraspItAccessor() {}

    // pointer to the GraspItSceneManager object. GraspItSceneManager is supposed to
    // be threadsafe, though at beta stage, no guarantees are given yet.
    const SHARED_PTR<GraspItSceneManager> graspItInterface;

    const std::string name;

    bool registered;

    bool scheduled;
    MUTEX scheduledMtx;
};
}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_GRASPITACCESSOR_H
