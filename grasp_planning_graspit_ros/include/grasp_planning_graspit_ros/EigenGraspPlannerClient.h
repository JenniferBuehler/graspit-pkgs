#ifndef GRASP_PLANNING_GRASPIT_ROS_EIGENGRASPPLANNERCLIENT_H
#define GRASP_PLANNING_GRASPIT_ROS_EIGENGRASPPLANNERCLIENT_H

#include <ros/ros.h>
#include <grasp_planning_graspit_msgs/LoadDatabaseModel.h>
#include <grasp_planning_graspit_msgs/AddToDatabase.h>
#include <manipulation_msgs/GraspPlanning.h>
#include <geometry_msgs/Pose.h>

namespace grasp_planning_graspit_ros
{

/**
 * \brief Uses the GraspIt! services to add and load robot and objects and do the planning.
 * This class can be used as a helper.
 *
 * Reads parameters from ROS parameter service as specified and documented in
 * ``rosed grasp_planning_graspit_ros EigenGraspPlannerClient.yaml``
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class EigenGraspPlannerClient
{
public:
    EigenGraspPlannerClient(); 
    ~EigenGraspPlannerClient();

    /**
     * \return true if client configured properly and should be working.
     */
    bool isOK();

    /**
     * Adds a robot to the database
     * \param filename the file with the GraspIt! robot definition
     * \param jointNames the names of the URDF joint names in the order they are specified in the graspit file.
     * \return >=0 ID of robot loaded
     * \retval -1 could not call service
     * \retval -2 could not add robot to database 
     * \retval -3 client not initialized. Have the ROS parameters been set for the service names?
     */
    int addRobot(const std::string& modelName, const std::string filename, const std::vector<std::string>& jointNames); 

    /**
     * Adds an object to the database
     * \param filename the file with the GraspIt! object definition
     * \param objectGraspable set to true is object is graspable
     * \return >=0 ID of object loaded
     * \retval -1 could not call service
     * \retval -2 could not add object to database
     * \retval -3 client not initialized. Have the ROS parameters been set for the service names?
     */
    int addObject(const std::string& modelName, const std::string filename, bool objectGraspable); 

    /**
     * Loads a model from the database to the GraspIt! world
     * \param modelID ID as returned from addRobot() or addObject() 
     * \param clearWorld clear other models before loading this one
     * \param modelPose the pose where to load the model.
     * \retval 0 success
     * \retval -1 failed to call service
     * \retval -2 could not load model, is it not in the database? 
     * \retval -3 client not initialized. Have the ROS parameters been set for the service names?
     */
    int loadModel(const int modelID, bool clearWorld, const geometry_msgs::Pose& modelPose); 

    /**
     * Does the planning for the given \e robotModelName and \e objectID. Both models have to be
     * loaded in the GraspIt! world (by function loadModel()).
     * \param objectID ID as returned from addRobot() or addObject()
     * \param newObjectPose this is optional and can be set to NULL to disable.
     *      Set a different pose to put the object at in the current graspit world.
     *      Has to be in the global frame of the graspit world.
     *      If NULL, the current pose in the GraspIt! pose will be used.
     * \param resultsOutputDirectory the directory where the Grasp messages will be saved. This
     *      does not need to be the same directory at which the GraspIt! service stores its grasp results.
     *      Set to empty string if results should not be saved to file but only returned in \e results
     * \retval 0 planning successfully done, results written to output directory
     * \retval -1 could not create results directory (only tried if \e resultsOutputDirectory non-empty)
     * \retval -2 could not call planning service
     * \retval -3 planning failed
     * \retval -4 client not initialized. Have the ROS parameters been set for the service names?
     */
    int plan(const std::string robotModelName, const int objectID,
        const geometry_msgs::Pose * newObjectPose, const std::string& resultsOutputDirectory,
        std::vector<manipulation_msgs::Grasp>& results); 

private:

    bool saveToFile(const manipulation_msgs::Grasp& msg, const std::string& filename, bool asBinary); 
    bool makeDirectoryIfNeeded(const std::string& dPath); 
    void init(); 

    bool initialized;
    std::string addToDBService;
    std::string loadModelService;
    std::string saveWorldService;
    std::string egPlanningService;

    ros::NodeHandle node;
    ros::ServiceClient add_to_db_client, load_model_client, eg_planner_client;


};
}  // namespace
#endif  // GRASP_PLANNING_GRASPIT_ROS_EIGENGRASPPLANNERCLIENT_H
