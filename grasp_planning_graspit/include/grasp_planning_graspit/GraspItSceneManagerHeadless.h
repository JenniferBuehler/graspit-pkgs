#ifndef GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGERNOGUI_H
#define GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGERNOGUI_H

#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Scene Manager for the GraspIt world which does not require the GUI.

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
#include <grasp_planning_graspit/GraspItSceneManager.h>

class SoIdleSensor;

namespace GraspIt
{

/**
 * \brief Provides scene manager which can be used to load and access the GraspIt! world without the need to run the GUI.
 *
 * In this initial implementation, this interface will be running the **SoQt** loop in a separate thread.
 * This is also the scene manager thread to which GraspItAccessor instances may get access to via GraspItAccessor::addAsIdleListener().
 *
 * Subclasses which want to make use of QT signals and slots, or any other mechanisms proivded by QObject and
 * the Qt metaobject system, will have to additionally derive from QObject, and the MOC files have to be generated for it.
 *
 * **Important note:** While this implementation is still using Qt, you **may not** touch any Qt stuff before creating an intance
 * of this class. This means, you cannot create any QObject instances either. Otherwise you will get the Qt warning
 * "WARNING: QApplication was not created in the main() thread". Mostly this should not matter, but it could cause
 * problems, e.g. with QPixmap, if you are using them.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GraspItSceneManagerHeadless: public GraspItSceneManager
{
public:
    GraspItSceneManagerHeadless();
    virtual ~GraspItSceneManagerHeadless();

    virtual bool isReady() const;

    virtual void waitUntilReady() const;

protected:
    virtual void initializeCore();

    virtual void destroyCore();

    virtual World * createNewGraspitWorld();

    virtual bool eventThreadRunsQt() const
    {
        return true;
    }


    /**
     * Simulates the creation of an "Idle Event" by scheduling mIdleSensor (if it is not already scheduled).
     *
     * \retval false Inventor is not ready yet.
     */
    virtual bool scheduleIdleEvent();

    /**
     * returns ivReady flag
     */
    bool isInventorReady() const;

private:
    GraspItSceneManagerHeadless(const GraspItSceneManagerHeadless& o) {}

    /**
     * Creates the SoIdleSensor object which will keep running from the Inventor thread and schedules it.
     * Should be called from within the thread that runs the inventor loop.
     */
    void createIdleSensor();
    void deleteIdleSensor();

    /**
     * the SoIdleSensor callback function
     */
    static void sensorCB(void *data, SoSensor *);

    /**
     * Before any methods are called which rely on the Inventor stuff to run, this
     * method can be called with true to wait for the initialization of Inventor to be ready.
     * If the method needs to wait until Inventor is shut down, call this method with false.
     */
    void waitForInventorState(const bool value) const;

    /**
     * Threadsafe method to set flag ivReady
     */
    void setInventorReady(const bool flag);

    /**
     * Method for thread to run the inventor manager main loop (CoreHeadless::beginMainLoop()).
     * This thread callback also needs to be used to initialize the CoreHeadless, because initialisation
     * and main loop have to be handled by the same thread.
     */
    static void ivThreadLoop(GraspItSceneManagerHeadless * _this);

    // Thread which runs the inventor loop.
    THREAD * ivThread;

    // flag set to true as soon as the Inventor environment managed by CoreHeadless is initialized.
    bool ivReady;
    mutable MUTEX ivReadyMtx;

    SoSensor *mIdleSensor;
};
}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_GRASPITSCENEMANAGERNOGUI_H
