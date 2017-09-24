#ifndef GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNER_H
#define GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNER_H

/**
   Interface to the GraspIt planning algorithms.

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
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/GraspItAccessor.h>

#include <vector>
#include <string>

#include <QObject>

#define DEFAULT_MAX_PLANNING_STEPS 70000
#define DEFAULT_MAX_RESULTS_PER_REPEAT 10
#define DEFAULT_FINISH_WITH_AUTOGRASP false

// To use  during beta phase, until decided which implementation
// is better.
// If this is defined, the implementation uses its own local
// SoIdleSensor to repeatedly call ivIdleCallback(). If
// this is not defined, the GraspItSceneManager event loop
// callback is used to call ivIdleCallback().
// #define USE_SEPARATE_SOSENSOR

class GraspPlanningState;
class GraspableBody;
class EGPlanner;
class SearchEnergy;
class GraspPlanningState;
class GraspItAccessor;

namespace GraspIt
{

/**
 * \brief Runs the GraspIt! Eigengrasp planning algorithm.
 *
 * This class extends the GraspItAccessor class to run the Eigengrasp planning algorithms
 * in the original graspit source on the world.
 *
 * In the current implementation, Qt is required for running the Eigengrasp planner.
 * GraspItSceneManager::eventThreadRunsQt() needs to return true for the GraspItSceneManager
 * passed into the constructor of this class. For an example on how to implement this class
 * without direct Qt dependencies, check out EigenGraspPlannerNoQt.
 *
 * *Notes*: See also [git issue #1](https://github.com/JenniferBuehler/graspit-pkgs/issues/1) for restrictions of this class.
 *
 * *Important note for developers*: This class should not be derived further in its current implementation.
 * This is because it uses an SoIdleSensor locally which it destroys in the destructor. This needs to be destroyed *before*
 * the mandatory call of removeFromIdleListeners() in all subclasses destructors.
 * Support for further extension is easy to achieve (by not using the local SoIdleSensor or adding a new method
 * shutdownEigenGraspPlanner() or similar), but it has not been considered at this stage yet.
 *
 * \author Jennifer Buehler
 * \date December 2015
 */
class EigenGraspPlanner: public QObject, public GraspItAccessor
{
    Q_OBJECT

public:
    // So far, only AxisAngle supported, as others not tested yet.
    // Later will be: enum  GraspItStateType{Complete, AxisAngle, Ellipsoid, Approach};
    enum GraspItStateType  {AxisAngle};

    // Mapping to GraspIt! search energy type. For now only
    // supports CONTACT_ENERGY. All other types still to be added:
    // CONTACT_ENERGY, POTENTIAL_QUALITY_ENERGY, CONTACT_QUALITY_ENERGY,
    // AUTOGRASP_QUALITY_ENERGY, GUIDED_AUTOGRASP_ENERGY, STRICT_AUTOGRASP_ENERGY,
    // COMPLIANT_ENERGY, DYNAMIC_ENERGY
    enum GraspItSearchEnergyType  {EnergyContact};

    // Type of planner ot use. So far, only simulated annealing supported, as
    // others are not tested.
    // TODO: implement other planner types as well.
    // Later this should be: enum PlannerType {SimAnn, Loop, MultiThreaded };
    enum PlannerType {SimAnn};

    /**
     * Creates and initializes the planner. Also registers the class to enable updates
     * from the scene manager thread by calling addAsSchedulable(). If
     * eventThreadRunsQt() returns false, an exception is thrown, because this implementation
     * requires updates from the main Qt loop.
     */
    EigenGraspPlanner(const std::string& name, const SHARED_PTR<GraspItSceneManager>& i);
    virtual ~EigenGraspPlanner();

    /**
     * Starts planning with the specified planner type. NOTE: Only simulated annealing is tested so far.
     * \see void EigenGraspPlannerDlg::startPlanner() and void EigenGraspPlannerDlg::plannerStart_clicked()
     * \param maxPlanningSteps maximum steps (iterations) for the planning algorithm to use
     * \param planType the type of planning algorithm to use. To this point, only simulated annealing is supported.
     * \param repeatPlanning number of times the planning process should be repeated. This would in effect
     *                       be the same as calling this method <repeatPlanning> times and then pick the best
     *                       results.
     * \return false if planner could not be initialized or if it failed.
     */
    bool plan(const int maxPlanningSteps,// = DEFAULT_MAX_PLANNING_STEPS,
              const int repeatPlanning,// = 1,
              const int maxResultsPerRepeat,// = DEFAULT_MAX_RESULTS_PER_REPEAT,
              const bool finishWithAutograsp,// = DEFAULT_FINISH_WITH_AUTOGRASP,
              const PlannerType& planType = SimAnn);

    /**
     * Sets the current hand and object before calling plan(const int, const int).
     * This is achieved by calling GraspItSceneManager::setCurrentHand(std::string&) and
     * GraspItSceneManager::setCurrentGraspableObject(std::string&), so it involves checking if these objects
     * are currently loaded in the GraspIt world. And if they are not, this method returns false.
     *
     * Aside from providing the possibility to change current hand and object, it is also possible
     * to specify a new object pose, if the parameter is not NULL (otherwise the current pose will be used).
     *
     * This method is threadsafe in regards to world changes. So if thread safety is a concern,
     * you should rather use this method rather than first calling
     * GraspItSceneManager::setCurrentHand() and then GraspItSceneManager::setCurrentGraspableObject()
     * before calling plan(const int, const int).
     * \param maxPlanningSteps maximum steps (iterations) for the planning algorithm to use
     * \param planType the type of planning algorithm to use. To this point, only simulated annealing is supported.
     * \param repeatPlanning number of times the planning process should be repeated. This would in effect
     *                       be the same as calling this method <repeatPlanning> times and then pick the best
     *                       results.
     *
     * \param maxResultsPerRepeat keep the n best results of each planning run.
     *      This number multiplied by \e repeatPlanning is the maximum
     *      number of resulting grasps for a call fo the service.
     * \param finishWithAutograsp Each result is 'finalized' with an additional Auto-Grasp,
     *      to ensure fingers are really closed around the object.
     *      Sometimes, a resulting grasp does not really touch the
     *      object, so setting this to true can help to ensure there
     *      are actual contact points.
     * \return false if planner could not be initialized or if it failed.
     */
    bool plan(const std::string& handName, const std::string& objectName,
              const EigenTransform * objectPose,
              const int maxPlanningSteps,//  = DEFAULT_MAX_PLANNING_STEPS,
              const int repeatPlanning,//   = 1
              const int maxResultsPerRepeat,// = DEFAULT_MAX_RESULTS_PER_REPEAT,
              const bool finishWithAutograsp,// = DEFAULT_FINISH_WITH_AUTOGRASP,
              const PlannerType& planType = SimAnn);

    /**
     * Saves the results as a GraspIt world files and/or inventor files in the given directory,
     * with the given filename prefix.
     * The files can be used to display the results with the original GraspIt simulator GUI or
     * an Inventor files viewer.
     * \param createDir if true, the directory in which the files are to be saved is created if it does not exist.
     * \param asGraspIt if true, the world files will be saved in the GraspIt world format
     * \param asInventor if true, the world files will be saved in the Inventor format.
     * Saving as Inventor is slower than as GraspIt.
     * \param saveSeparatePoseIV if true, (additional) inventor files will be saved with the robot poses (without the object).
     * \return false if creating the directory or writing the world files failed.
     */
    bool saveResultsAsWorldFiles(const std::string& inDirectory, const std::string& fileNamePrefix,
                                 bool asGraspIt = true, bool asInventor = false, bool createDir = true,
                                 bool saveSeparatePoseIV = false);

    /**
     * Returns the results of the grasp planning as EigenGraspResult objects.
     */
    void getResults(std::vector<EigenGraspResult>& allGrasps) const;

protected:
    virtual void idleEventFromSceneManager();

    virtual void onSceneManagerShutdown();

private:
    /**
     * Deletes the objects in the temporary result buffer (field \e results)
     * and clears the array.
     */
    void deleteResults();

    /**
     * Copies relevant fields from \e s to \e result
     * \return false if state \e s is invalid
     */
    bool copyResult(const GraspPlanningState * s, EigenGraspResult& result) const;

    /**
     * Method which will be called at regular intervals from the thread which is also
     * running the SoQt loop.
     */
    void ivIdleCallback();

    /**
     * initializes the planner to use the specified planning type. This method needs to be called before
     * the GraspIt planner is started.
     * \see void MainWindow::eigenGraspEigenActivated()
     * \param maxPlanningSteps maximum steps (iterations) for the planning algorithm to use
     * \param planType the type of planning algorithm to use. To this point, only simulated annealing is supported.
     */
    bool initPlanner(const int maxPlanningSteps, const PlannerType& planType = SimAnn);

    /**
     * \see void EigenGraspPlannerDlg::stopPlanner()
     */
    // void stopPlanner();


    /**
     * Iniitalizes the search type to use. Note: Only SPACE_AXIS_ANGLE tested here so far.
     * \param stateTemplate tells the planner how the state it is searching on looks like (how many variables, etc).
     * This object is initialized in this function according to the state type and the current hand and the
     * object set to grasp.
     * \see void EigenGraspPlannerDlg::spaceSearchBox_activated( const QString &s )
     */
    void initSearchType(GraspPlanningState& stateTemplate,  const GraspItStateType& st);

    /**
     * \param stateTemplate tells the planner how the state it is searching on looks like (how many variables, etc).
     * \see void EigenGraspPlannerDlg::plannerInit_clicked()
     */
    void initPlannerType(const GraspPlanningState& stateTemplate, const PlannerType& pt);


    /**
     * \see void EigenGraspPlannerDlg::updateResults(bool render)
     */
    void updateResults();

    /**
     * To be called at regular intervals to display the current state of the planning process
     * \see void EigenGraspPlannerDlg::plannerUpdate()
     */
    void plannerUpdate();


    /**
     * To be called when the planning is finished to display the final results
     * \see void EigenGraspPlannerDlg::plannerComplete()
     */
    void plannerComplete();


    /**
     * Sets the planning parameters as specified in the fields graspitSearchEnergyType and
     * useContacts
     * \see void EigenGraspPlannerDlg::readPlannerSettings()
     */
    void setPlanningParameters();

    /**
     * \see void EigenGraspPlannerDlg::bestGraspButton_clicked()
     */
    void printPlanningResults();

    /**
     * \see void EigenGraspPlannerDlg::plannerReset_clicked()
     */
    void plannerReset();

    /**
     * Load an input grasp file (".txt")
     * TODO TEST ME
     * \see void EigenGraspPlannerDlg::inputLoadButton_clicked()
     */
    // void inputLoad(const std::string& inputGraspFile);

    /**
     * prints the result of the i'th grasp found. While the planning is ongoing, this method
     * should be called from within the planner thread to ensure thread safety.
     */
    void printResult(int i, bool detailed = false);

    // double getEnergy(const GraspPlanningState * s);

    /**
     * Gets the transform of the Hand in a search state
     */
    EigenTransform getHandTransform(const GraspPlanningState * s) const;
    /**
     * Gets the transform of the object in a search state
     */
    EigenTransform getObjectTransform(const GraspPlanningState * s) const;

    /**
     * Check if this is a state that the current implementation can work with
     */
    bool checkStateValidity(const GraspPlanningState * s) const;

    /**
     * Gets the DOFs of the joints for the grasp of this state
     */
    void getGraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const;

    /**
     * Gets the DOFs of the joints for the pre-grasp of this state. This will execute
     * an auto-grasp to open the hand before reading the DOF values.
     */
    void getPregraspJointDOFs(const GraspPlanningState * s, std::vector<double>& dofs) const;

    /**
     * gets the Eigengrasp values associated with this grasp state
     */
    void getEigenGraspValues(const GraspPlanningState * s, std::vector<double>& egVals) const;


#ifdef USE_SEPARATE_SOSENSOR
    /**
     * Callback method for locally used SoIdleSensor
     */
    static void sensorCB(void *data, SoSensor *);
#endif

public slots:
    /**
     * Qt slot which the EGPlanner class in the original graspit source can use
     * to notify this class about updates in the planning process.
     */
    void plannerUpdateSlot();
    /**
     * Qt slot which the EGPlanner class in the original graspit source can use
     * to notify this class when the planning process is completed.
     */
    void plannerCompleteSlot();

private:
    enum PlannerCommand {NONE, START, STOP};

    /**
     * Threadsafe method to read planCommand
     */
    PlannerCommand getPlannerCommand();
    /**
     * Threadsafe method to set planCommand
     */
    void setPlannerCommand(const PlannerCommand c);

    PlannerCommand planCommand;
    MUTEX planCommandMtx;

    // for test purposes: to separately calculate energy of grasps
    // SearchEnergy * mEnergyCalculator;

    EGPlanner * graspitEgPlanner;
    RECURSIVE_MUTEX graspitEgPlannerMtx;

    // The state type to use for GraspIt. Default is STATE_AXIS_ANGLE.
    GraspItStateType graspitStateType;

    // The search energy type to use. Is ENERGY_CONTACT by default.
    // Important: ENERGY_CONTACT_QUALITY doesn't work properly at the moment,
    // at least not with the Jaco hand, also not in the original simulator.... the energy
    // drops to -0. Reason to be found in SearchEnergy::potentialQualityEnergy, variable gq
    // gets 0. So far, only successfully tested method was ENERGY_CONTACT, though I didn't
    // test all others except ENERGY_CONTACT_QUALITY
    GraspItSearchEnergyType graspitSearchEnergyType;

    // whether to use contacts information in the planner or not
    bool useContacts;

    std::vector<const GraspPlanningState*> results;

#ifdef USE_SEPARATE_SOSENSOR
    SoSensor *mIdleSensor;
#endif
};

}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNER_H
