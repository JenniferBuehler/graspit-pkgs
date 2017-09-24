#ifndef GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNERNOQT_H
#define GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNERNOQT_H

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

#define DEFAULT_MAX_PLANNING_STEPS 70000

#include <EGPlanner/search.h>

class GraspPlanningState;
class GraspableBody;
class EGPlanner;
class SearchEnergy;
class GraspPlanningState;
class GraspItAccessor;

namespace GraspIt
{

/**
 * \brief Runs the GraspIt! Eigengrasp planning algorithm without using Qt signals/slots.
 *
 * This class extends the GraspItAccessor class to run the Eigengrasp planning algorithms
 * in the original graspit source on the world. It is an example implementation of how
 * access to the original graspit eigengrasp planner source without requiring direct
 * Qt dependencies from within this class, as is the case with class EigenGraspPlanner.
 *
 * *Notes*:
 * - At this stage, only support for simmulated annealing planning types is offered.
 *      "Loop" and "multi-threaded" can probably be integrated fairly easy. So far, no
 *       support for "on-line" has been considered.
 * -  Moreover, only the state description type STATE_AXIS_ANGLE (GraspIt datatype StateType) has been tested, and only
 *      the search energy type "CONTACT_ENERGY" (as defined in GraspIt).
 *      Problems were encountered with ENERGY_CONTACT_QUALITY (also with the original GraspIt simulator) when testing with
 *      the Jaco hand (see comment for field graspitSearchEnergyType).
 *      Because of this, at the moment there is no public method to change the state or energy type. By default,
 *      STATE_AXIS_ANGLE and ENERGY_CONTACT are used.
 *
 * \author Jennifer Buehler
 * \date December 2015
 */
class EigenGraspPlannerNoQt: public GraspItAccessor
{
public:
    // So far, only AxisAngle supported, as others not tested yet.
    // Later: enum SearchType {Complete, AxisAngle, Ellipsoid, Approach};
    // enum SearchType  {AxisAngle};

    // Type of planner ot use. So far, only simulated annealing supported, as
    // others are not tested.
    // TODO: implement other planner types as well.
    // Later this should be: enum PlannerType {SimAnn, Loop, MultiThreaded };
    enum PlannerType {SimAnn};

    /**
     * Creates and initializes the planner.
     */
    EigenGraspPlannerNoQt(const std::string& name, const SHARED_PTR<GraspItSceneManager>& i);
    virtual ~EigenGraspPlannerNoQt();

    /**
     * Starts planning with the specified planner type. NOTE: Only simulated annealing is tested so far.
     * \see void EigenGraspPlannerNoQtDlg::startPlanner() and void EigenGraspPlannerNoQtDlg::plannerStart_clicked()
     * \param maxPlanningSteps maximum steps (iterations) for the planning algorithm to use
     * \param planType the type of planning algorithm to use. To this point, only simulated annealing is supported.
     * \return false if planner could not be initialized or if it failed.
     */
    bool plan(const int maxPlanningSteps = DEFAULT_MAX_PLANNING_STEPS, const PlannerType& planType = SimAnn);

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
     */
    bool plan(const std::string& handName, const std::string& objectName,
              const EigenTransform * objectPose = NULL,
              const int maxPlanningSteps = DEFAULT_MAX_PLANNING_STEPS, const PlannerType& planType = SimAnn);

    /**
     * Saves the results as a GraspIt world files and/or inventor files in the given directory,
     * with the given filename prefix.
     * The files can be used to display the results with the original GraspIt simulator GUI or
     * an Inventor files viewer.
     * \param createDir if true, the directory in which the files are to be saved is created if it does not exist.
     * \param asGraspIt if true, the world files will be saved in the GraspIt world format
     * \param asInventor if true, the world files will be saved in the Inventor format.
     * Saving as Inventor is slower than as GraspIt.
     * \return false if creating the directory or writing the world files failed.
     */
    bool saveResultsAsWorldFiles(const std::string& inDirectory, const std::string& fileNamePrefix,
                                 bool asGraspIt = true, bool asInventor = false, bool createDir = true);

    /**
     * Returns the results of the grasp planning as EigenGraspResult objects.
     */
    void getResults(std::vector<EigenGraspResult>& allGrasps) const;

protected:
    virtual void idleEventFromSceneManager();

    virtual void onSceneManagerShutdown();

private:
    /**
     * initializes the planner to use the specified planning type. This method needs to be called before
     * the GraspIt planner is started.
     * \see void MainWindow::eigenGraspEigenActivated()
     * \param maxPlanningSteps maximum steps (iterations) for the planning algorithm to use
     * \param planType the type of planning algorithm to use. To this point, only simulated annealing is supported.
     */
    bool initPlanner(const int maxPlanningSteps, const PlannerType& planType = SimAnn);

    /**
     * \see void EigenGraspPlannerNoQtDlg::stopPlanner()
     */
    // void stopPlanner();


    /**
     * Iniitalizes the search type to use. Note: Only SPACE_AXIS_ANGLE tested here so far.
     * \param stateTemplate tells the planner how the state it is searching on looks like (how many variables, etc).
     * This object is initialized in this function according to the state type and the current hand and the
     * object set to grasp.
     * \see void EigenGraspPlannerNoQtDlg::spaceSearchBox_activated( const QString &s )
     */
    void initSearchType(GraspPlanningState& stateTemplate,  const StateType& st);

    /**
     * \param stateTemplate tells the planner how the state it is searching on looks like (how many variables, etc).
     * \see void EigenGraspPlannerNoQtDlg::plannerInit_clicked()
     */
    void initPlannerType(const GraspPlanningState& stateTemplate, const PlannerType& pt);


    /**
     * \see void EigenGraspPlannerNoQtDlg::updateResults(bool render)
     */
    void updateResults();

    /**
     * To be called at regular intervals to display the current state of the planning process
     * \see void EigenGraspPlannerNoQtDlg::plannerUpdate()
     */
    void plannerUpdate();


    /**
     * To be called when the planning is finished to display the final results
     * \see void EigenGraspPlannerNoQtDlg::plannerComplete()
     */
    void plannerComplete();


    /**
     * Sets the planning parameters as specified in the fields graspitSearchEnergyType and
     * useContacts
     * \see void EigenGraspPlannerNoQtDlg::readPlannerSettings()
     */
    void setPlanningParameters();

    /**
     * \see void EigenGraspPlannerNoQtDlg::bestGraspButton_clicked()
     */
    void printPlanningResults();

    /**
     * \see void EigenGraspPlannerNoQtDlg::plannerReset_clicked()
     */
    void plannerReset();

    /**
     * Load an input grasp file (".txt")
     * TODO TEST ME
     * \see void EigenGraspPlannerNoQtDlg::inputLoadButton_clicked()
     */
    // void inputLoad(const std::string& inputGraspFile);

    /**
     * prints the result of the i'th grasp found. While the planning is ongoing, this method
     * should be called from within the planner thread to ensure thread safety.
     */
    void printResult(int i, bool detailed = false);

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

    static void statusThreadLoop(EigenGraspPlannerNoQt * _this);

    // thread that prints the status messages during planning process
    THREAD * statusThread;


    // for test purposes: to separately calculate energy of grasps

    EGPlanner * graspitEgPlanner;
    RECURSIVE_MUTEX graspitEgPlannerMtx;

    // The state type to use for GraspIt. Default is STATE_AXIS_ANGLE.
    StateType graspitStateType;

    // The search energy type to use. Is CONTACT_ENERGY by default.
    // Important: ENERGY_CONTACT_QUALITY doesn't work properly at the moment,
    // at least not with the Jaco hand, also not in the original simulator.... the energy
    // drops to -0. Reason to be found in SearchEnergy::potentialQualityEnergy, variable gq
    // gets 0. So far, only successfully tested method was CONTACT_ENERGY, though I didn't
    // test all others except ENERGY_CONTACT_QUALITY
    std::string graspitSearchEnergyType;

    // whether to use contacts information in the planner or not
    bool useContacts;

    std::vector<const GraspPlanningState*> results;
};

}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_EIGENGRASPPLANNERNOQT_H
