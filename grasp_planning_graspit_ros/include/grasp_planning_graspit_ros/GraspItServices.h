#ifndef GRASP_PLANNING_GRASPIT_ROS_GRASPITSERVICES_H
#define GRASP_PLANNING_GRASPIT_ROS_GRASPITSERVICES_H

/**
   Provides ROS services to access functionality of grasp_planning_graspit.

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


#include <ros/ros.h>
#include <string>
#include <vector>

#include <grasp_planning_graspit/SharedPtr.h>

#include <grasp_planning_graspit_msgs/LoadDatabaseModel.h>
#include <grasp_planning_graspit_msgs/AddToDatabase.h>
#include <grasp_planning_graspit_msgs/SaveWorld.h>

#include <manipulation_msgs/GraspPlanning.h>

namespace GraspIt
{

class GraspItSceneManager;
class GraspItSimpleDBManager;
class EigenGraspPlanner;
class EigenGraspResult;

/**
 * \brief Provides ROS services to access functionality of package grasp_planning_graspit.
 *
 * This class maintains a GraspIt::GraspItDatabaseManager object. Services are provided to:
 * - add objects to the database with *grasp_planning_graspit_msgs/AddToDatabase.srv*
 * - to load them to the current %GraspIt world with *grasp_planning_graspit_msgs/LoadDatabaseModel.srv*.
 * - to save the currently loaded %GraspIt world, with the
 *   service type *grasp_planning_graspit_msgs/SaveWorld.srv*.
 * - accepts grasp planning requests with the service type *manipulation_msgs/GraspPlanning.srv*.
 *
 * The class GraspIt::EigenGraspPlanner is used for the planning.
 * At this stage, the GraspIt::EigenGraspPlanner implementation supports only simulated annealing.
 *
 * About the grasp planning request:
 * - The service calls the GraspIt! planner assuming that all object and robot poses are
 *      specified in the **global frame** (the graspit world origin).
 * - The frame id used in the geometry_msgs::PoseStamped objects have to be either "0" (pose not used)
 *   or "1" (pose object used in global frame). No tf transforms are supported yet.
 *
 * About the results of a grasp planning request:
 * - The resulting manipulation_msgs::Grasp messages return the grasp pose **relative to the object**.
 * - Grasp quality values returned are the energy of the simulated annealing process.
 * - Pre-grasp joint states (see manipulation_msgs::Grasp::pre_grasp_posture)
 *   are not supported yet. It is assumed that the hand is plain open in pre-grasp
 *   stage for now.
 * - No joint velocities and efforts in the resulting sensor_msgs::JointState objects are provided.
 *
 *
 * All parameters have to be specified within the node's private namespace. Parmeters are
 * documented in the .yaml config file:
 * ``rosed grasp_planning_graspit_ros GraspItServices.yaml`` 
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GraspItServices
{
public:
    GraspItServices();
    ~GraspItServices();

    /**
     * Start the services.
     */
    void start();

private:
    GraspItServices(const GraspItServices& o) {}

    void init();

    // reads parameters from the parameter service
    void readParams();

    /**
     * callback method for service
     */
    bool acceptAddToDB(grasp_planning_graspit_msgs::AddToDatabase::Request &req,
                       grasp_planning_graspit_msgs::AddToDatabase::Response &res);

    /**
     * callback method for service
     */
    bool acceptLoadModel(grasp_planning_graspit_msgs::LoadDatabaseModel::Request &req,
                         grasp_planning_graspit_msgs::LoadDatabaseModel::Response &res);

    /**
     * callback method for service
     */
    bool acceptSaveWorld(grasp_planning_graspit_msgs::SaveWorld::Request &req,
                         grasp_planning_graspit_msgs::SaveWorld::Response &res);

    /**
     * callback method for service
     */
    bool acceptEGPlanning(manipulation_msgs::GraspPlanning::Request &req,
                          manipulation_msgs::GraspPlanning::Response &res);

    /**
     * Helper method to extract results from an GraspIt::EigenGraspResult object
     * and create a manipulation_msgs::Grasp object from it.
     * \param objectFrame the object's reference frame to use for putting into the
     * resulting manipulation_msgs::Grasp (typically the center of the object)
     */
    manipulation_msgs::Grasp getGraspMsg(const EigenGraspResult& r, const std::string& id,
                                         const std::vector<std::string>& robotJointNames,
                                         const std::string& objectFrame) const;


    SHARED_PTR<GraspItSceneManager> graspitMgr;
    SHARED_PTR<GraspItSimpleDBManager> mgr;
    SHARED_PTR<GraspIt::EigenGraspPlanner> egPlanner;

    // ---- service servers ---

    ros::ServiceServer addGraspitFileSrv;
    ros::ServiceServer loadGraspitModelSrv;
    ros::ServiceServer saveWorldSrv;
    ros::ServiceServer egPlanningSrv;

    // --- variables read from ROS parameter server (commented in class header)

    std::string dbName;
    std::string egPlannerName;
    std::string addToDBTopic;
    std::string loadModelTopic;
    std::string saveWorldTopic;
    std::string egPlanningTopic;
    int defaultMaxPlanningSteps;
    int defaultNumRepeatPlanning;
    int defaultNumKeepResults;
    bool defaultFinishWithAutograsp;
    float graspMsgPositionFactor;
    std::string resultsOutputDirectory;
    bool saveResultFilesInventor;
    bool saveResultFilesGraspit;


    // if true, the joint DOF values will be generated. This may be necessary
    // because graspit generates results with opposite values
    bool negateJointDOFs;


    ros::NodeHandle priv;
    ros::NodeHandle pub;
};

}  //  namespace GraspIt

#endif  // GRASP_PLANNING_GRASPIT_ROS_GRASPITSERVICES_H
