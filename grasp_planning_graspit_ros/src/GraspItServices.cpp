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

#include <grasp_planning_graspit_ros/GraspItServices.h>

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/GraspItSimpleDBManager.h>

#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <grasp_planning_graspit/SharedPtr.h>

#include <eigen_conversions/eigen_msg.h>

#include <string>
#include <algorithm>
#include <vector>

using GraspIt::GraspItServices;
using GraspIt::GraspItSceneManager;
using GraspIt::GraspItSimpleDBManager;
using GraspIt::EigenGraspPlanner;

#define DEFAULT_DB_NAME "DefaultDatabaseName"
#define DEFAULT_EGPLANNER_NAME "graspitEGPlanner"

#define DEFAULT_ADD_DB_TOPIC "graspit_add_to_db"
#define DEFAULT_LOAD_MODEL_TOPIC "graspit_add_to_db"
#define DEFAULT_SAVE_WORLD_TOPIC "graspit_save_world"
#define DEFAULT_EGPLANNING_TOPIC "graspit_eg_planning"
#define DEFAULT_MAX_EGPLANNING_STEPS 70000
#define DEFAULT_NUM_REPEAT_PLANNING 1
#define DEFAULT_GRASP_MSG_POSITION_FACTOR 0.001
#define DEFAULT_NUM_KEEP_RESULTS 3
#define DEFAULT_FINISH_WITH_AUTOGRASP false
#define DEFAULT_OUTPUT_DIRECTORY ""
#define DEFAULT_SAVE_RESULT_FILES false
#define DEFAULT_NEGATE_JOINT_DOFS true

GraspItServices::GraspItServices():
    priv("~"),
    pub(""),
    negateJointDOFs(DEFAULT_NEGATE_JOINT_DOFS)
{
    init();
}



GraspItServices::~GraspItServices()
{
    PRINTMSG("Deleting GraspItServices");
}

void GraspItServices::init()
{
    readParams();

    PRINTMSG("Creating graspit interface and database");

    graspitMgr = SHARED_PTR<GraspItSceneManager>(new GraspItSceneManagerHeadless());

    mgr = SHARED_PTR<GraspItSimpleDBManager>(new GraspItSimpleDBManager(dbName, graspitMgr));
    egPlanner = SHARED_PTR<EigenGraspPlanner>(new EigenGraspPlanner(egPlannerName, graspitMgr));
}

void GraspItServices::readParams()
{
    priv.param<std::string>("database_name", dbName, DEFAULT_DB_NAME);
    PRINTMSG("Using database name: " << dbName);
    priv.param<std::string>("eg_planner_name", egPlannerName, DEFAULT_EGPLANNER_NAME);
    PRINTMSG("Using eg planner name: " << egPlannerName);
    priv.param<std::string>("add_to_db_topic", addToDBTopic, DEFAULT_ADD_DB_TOPIC);
    PRINTMSG("Using add_to_db_topic: " << addToDBTopic);
    priv.param<std::string>("load_model_topic", loadModelTopic, DEFAULT_LOAD_MODEL_TOPIC);
    PRINTMSG("Using load_model_topic: " << loadModelTopic);
    priv.param<std::string>("save_world_topic", saveWorldTopic, DEFAULT_SAVE_WORLD_TOPIC);
    PRINTMSG("Using save_world_topic: " << saveWorldTopic);
    priv.param<std::string>("eg_planning_topic", egPlanningTopic, DEFAULT_EGPLANNING_TOPIC);
    PRINTMSG("Using eg_planning_topic: " << egPlanningTopic);
    priv.param<int>("default_max_planning_steps", defaultMaxPlanningSteps, DEFAULT_MAX_EGPLANNING_STEPS);
    PRINTMSG("Using default max number of planning steps: " << defaultMaxPlanningSteps);
    priv.param<int>("default_num_repeat_planning", defaultNumRepeatPlanning, DEFAULT_NUM_REPEAT_PLANNING);
    PRINTMSG("Using default number of planning repeats: " << defaultNumRepeatPlanning);
    priv.param<float>("grasp_msg_position_factor", graspMsgPositionFactor, DEFAULT_GRASP_MSG_POSITION_FACTOR);
    PRINTMSG("Using default factor for position values: " << graspMsgPositionFactor);
    priv.param<int>("default_num_keep_results", defaultNumKeepResults, DEFAULT_NUM_KEEP_RESULTS);
    PRINTMSG("Using default number of results kept: " << defaultNumKeepResults);
    priv.param<bool>("default_finish_with_autograsp", defaultFinishWithAutograsp, DEFAULT_FINISH_WITH_AUTOGRASP);
    PRINTMSG("Finish with auto-grasp by default: " << defaultFinishWithAutograsp);
    priv.param<bool>("save_result_files_inventor", saveResultFilesInventor, DEFAULT_SAVE_RESULT_FILES);
    PRINTMSG("Save result files inventor: " << saveResultFilesInventor);
    priv.param<bool>("save_result_files_graspit", saveResultFilesGraspit, DEFAULT_SAVE_RESULT_FILES);
    PRINTMSG("Save result files graspit: " << saveResultFilesGraspit);
    priv.param<std::string>("results_output_directory", resultsOutputDirectory, DEFAULT_OUTPUT_DIRECTORY);
    PRINTMSG("Results output directory: " << resultsOutputDirectory);
}


void GraspItServices::start()
{
    addGraspitFileSrv = pub.advertiseService(addToDBTopic, &GraspItServices::acceptAddToDB, this);
    loadGraspitModelSrv = pub.advertiseService(loadModelTopic, &GraspItServices::acceptLoadModel, this);
    saveWorldSrv = pub.advertiseService(saveWorldTopic, &GraspItServices::acceptSaveWorld, this);
    egPlanningSrv = pub.advertiseService(egPlanningTopic, &GraspItServices::acceptEGPlanning, this);

    PRINTMSG("Ready to accept planning messages");
}

bool GraspItServices::acceptAddToDB(grasp_planning_graspit_msgs::AddToDatabase::Request &req,
                                    grasp_planning_graspit_msgs::AddToDatabase::Response &res)
{
//    PRINTMSG("Accepting add to database: "<<req);

    int modelID = -1;
    int ret = -1;
    if (req.isRobot)
    {
        if ((ret = mgr->loadRobotToDatabase(req.filename, req.modelName, req.jointNames)) < 0)
        {
            PRINTERROR("Could not load robot " << req.modelName << " from file "
                       << req.filename << ". Error code: " << ret);
        }
        else
        {
            PRINTMSG("Added robot " << req.modelName << " from file " << req.filename);
            modelID = ret;
        }
    }
    else
    {
        if ((ret = mgr->loadObjectToDatabase(req.filename, req.modelName, req.asGraspable)) < 0)
        {
            PRINTERROR("Could not load object " << req.modelName << " from file "
                       << req.filename << ". Error code: " << ret);
        }
        else
        {
            modelID = ret;
            PRINTMSG("Added object " << req.modelName << " from file " << req.filename << " and got ID=" << modelID);
        }
    }

    if (ret >= 0) res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::SUCCESS;
    else if (ret == -2) res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::NOT_READY;
    else if (ret == -3) res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::FILE_NOT_FOUND;
    else if (ret == -4) res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::MODEL_EXISTS;
    else if (ret == -5) res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::NO_NAME;
    else res.returnCode = grasp_planning_graspit_msgs::AddToDatabase::Response::OTHER_ERROR;

    res.modelID = modelID;

    return true;
}



bool GraspItServices::acceptLoadModel(grasp_planning_graspit_msgs::LoadDatabaseModel::Request &req,
                                      grasp_planning_graspit_msgs::LoadDatabaseModel::Response &res)
{
//    PRINTMSG("Accepting to load model: "<<req);

    int modelID = req.model_id;
    geometry_msgs::Pose modelPose = req.model_pose;
    bool clearOthers = req.clear_other_models;

    GraspIt::EigenTransform transform;
    tf::poseMsgToEigen(modelPose, transform);

    if (clearOthers)
    {
        PRINTMSG("Clearing other bodies before loading!");
        std::vector<int> ids;
        mgr->getAllLoadedRobotIDs(ids);
        mgr->getAllLoadedObjectIDs(ids);
        std::vector<int>::iterator it;
        for (it = ids.begin(); it != ids.end(); ++it)
        {
            int ret = mgr->unloadFromWorld(*it);
            if (ret != 0)
            {
                PRINTERROR("Could not unload model " << *it << " from world.");
                res.result = grasp_planning_graspit_msgs::LoadDatabaseModel::Response::LOAD_FAILURE;
                return true;
            }
        }
    }

    int ret = mgr->loadToWorld(modelID, transform);

    if (ret == 0)
    {
        PRINTMSG("Successfully loaded model " << modelID << " to the world.");
        res.result = grasp_planning_graspit_msgs::LoadDatabaseModel::Response::LOAD_SUCCESS;
    }
    else
    {
        PRINTERROR("Could not load model " << modelID << " to the world. Error code " << ret);
        res.result = grasp_planning_graspit_msgs::LoadDatabaseModel::Response::LOAD_FAILURE;
    }
    return true;
}

bool GraspItServices::acceptSaveWorld(grasp_planning_graspit_msgs::SaveWorld::Request &req,
                                      grasp_planning_graspit_msgs::SaveWorld::Response &res)
{
//    PRINTMSG("Accepting to save world: "<<req);

    const std::string inv = req.asInventor ? std::string(" as inventor") : std::string(" as graspit world");

    bool createDir = false;  // for safety, we'll keep the directory writing option disabled
    if (!mgr->saveLoadedWorld(req.filename, req.asInventor, createDir))
    {
        PRINTMSG("Could not save loaded world in file " << req.filename << inv);
        res.success = false;
    }
    else
    {
        PRINTMSG("Saved loaded world in file " << req.filename << inv);
        res.success = true;
    }

    return true;
}


manipulation_msgs::Grasp GraspItServices::getGraspMsg(const EigenGraspResult& egResult, const std::string& id,
        const std::vector<std::string>& robotJointNames, const std::string& objectFrame) const
{
    manipulation_msgs::Grasp g;
    g.id = id;

    sensor_msgs::JointState preJS;
    sensor_msgs::JointState graspJS;
    graspJS.name = robotJointNames;
    preJS.name = robotJointNames;
    std::vector<double> graspDOFs = egResult.getGraspJointDOFs();
    std::vector<double> pregraspDOFs = egResult.getPregraspJointDOFs();

    int nJoints = std::min(graspDOFs.size(), robotJointNames.size());

    if (graspDOFs.size() != robotJointNames.size())
    {
        PRINTERROR("Number of DOFs for the robot is not equal to number of joint names.");
        PRINTERROR("Therefore will only write the first " << nJoints << " joint values.");
    }
    if (graspDOFs.size() != pregraspDOFs.size())
    {
        PRINTERROR("Inconsistency: the number of joints in grasp should be equal to pre-grasp");
        pregraspDOFs=graspDOFs; // use the same state, so the rest of the code doesn't crash
    }

    graspJS.position.resize(nJoints, 0);
    preJS.position.resize(nJoints, 0);
    for (int i = 0; i < nJoints; ++i)
    {
        int mult = 1;
        if (negateJointDOFs) mult = -1;
        graspJS.position[i] = mult * graspDOFs[i];
        preJS.position[i] = mult * pregraspDOFs[i];
    }
    // not setting graspJS.header for the moment. Joint positions don't really need a reference
    // frame and we don't care about the timing...

    EigenTransform oToHand = egResult.getObjectToHandTransform();
    geometry_msgs::PoseStamped graspPose;
    tf::poseEigenToMsg(oToHand, graspPose.pose);
    graspPose.header.frame_id = objectFrame;
    graspPose.pose.position.x *= graspMsgPositionFactor;
    graspPose.pose.position.y *= graspMsgPositionFactor;
    graspPose.pose.position.z *= graspMsgPositionFactor;

    g.pre_grasp_posture = preJS;
    g.grasp_posture = graspJS;
    g.grasp_pose = graspPose;

    // TODO here we could compute a better quality value, and it depends on whether we have used
    // other algorithms than simulated annealing also... for now, return the simulated annealing
    // energy
    g.grasp_quality = egResult.getEnergy();

    // Approach and retreat not supported yet
    // g.approach=
    // g.retreat=

    // Max contact force disabled as not supported yet
    g.max_contact_force = -1;

    // Allowed touch objects not supported yet
    // g.allowed_touch_objects =
    return g;
}

bool GraspItServices::acceptEGPlanning(manipulation_msgs::GraspPlanning::Request &req,
                                       manipulation_msgs::GraspPlanning::Response &res)
{
//    PRINTMSG("Accepting to plan grasp: "<<req);

    // So far, this implementation only supports one potential model.
    if (req.target.potential_models.size() != 1)
    {
        PRINTERROR("In this implementation, only requests with only one object are supported.");
        res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;
        return true;
    }

    household_objects_database_msgs::DatabaseModelPose& dbObjModel = req.target.potential_models[0];

    // get the model ID for the robot and the name for the object
    std::string& robotName = req.arm_name;
    int robModelID = -1;
    if (!mgr->getRobotModelID(robotName, robModelID))
    {
        PRINTERROR("Could not get database ID for robot " << robotName);
        res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;
        return true;
    }
    std::string objectName;
    bool isRobot;
    if (!mgr->getModelNameAndType(dbObjModel.model_id, objectName, isRobot) || isRobot)
    {
        if (isRobot)
        {
            PRINTERROR("Model " << dbObjModel.model_id << " is a robot not an object");
        }
        else
        {
            PRINTERROR("Could not get name for object model " << dbObjModel.model_id);
        }
        res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;
        return true;
    }

    // Make sure we can get the robots joint names which we will need for the results
    std::vector<std::string> jointNames;
    if (!mgr->getRobotJointNames(robotName, jointNames) || jointNames.empty())
    {
        PRINTERROR("Could not get robot joint names, this is required for planning.");
        res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;
        return true;
    }

    // For now, object type information is not used.
    // This service looks up the object type itself.
    // ... = dbObjModel.type;


    SHARED_PTR<GraspIt::EigenTransform> objTransform;
    // If the reference frame is "0", use the object's current pose in the world.
    // Otherwise, change the model pose before planning.
    if (dbObjModel.pose.header.frame_id == "1")
    {
        PRINTMSG("Changing transform of the object:");
        PRINTMSG(dbObjModel.pose.pose);
        objTransform = SHARED_PTR<GraspIt::EigenTransform>(new GraspIt::EigenTransform());
        tf::poseMsgToEigen(dbObjModel.pose.pose, *objTransform);
    }

    // TODO in future we should provide options to change the planner type,
    // but at the moment only simulated annealing is supported
    EigenGraspPlanner::PlannerType planType = EigenGraspPlanner::SimAnn;

    // Would be nice to have a parameter in service request, bug GraspPlanning.srv
    // does not have this at the moment, so use default max planning steps.
    int maxPlanningSteps = defaultMaxPlanningSteps;

    PRINTMSG("Now starting the planning process...");
    if (!egPlanner->plan(robotName, objectName, objTransform.get(), maxPlanningSteps,
            defaultNumRepeatPlanning, defaultNumKeepResults, defaultFinishWithAutograsp))
    {
        PRINTERROR("Could not plan.");
        res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;
        return true;
    }

    if (saveResultFilesInventor || saveResultFilesGraspit)
    {
        PRINTMSG("Saving result world files in " << resultsOutputDirectory);
        std::stringstream filenamePrefix;
        filenamePrefix << robotName << "_" << objectName;
        egPlanner->saveResultsAsWorldFiles(resultsOutputDirectory, filenamePrefix.str(),
                                           saveResultFilesGraspit, saveResultFilesInventor);
    }

    std::vector<GraspIt::EigenGraspResult> allGrasps;
    egPlanner->getResults(allGrasps);

    PRINTMSG("Generating result grasp messages...");
    int i = 0;
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        // PRINTMSG(*it);
        std::stringstream id;
        id << robotName << "_" << objectName << "_" << i;
        manipulation_msgs::Grasp g = getGraspMsg(*it, id.str(), jointNames, dbObjModel.pose.header.frame_id);
        res.grasps.push_back(g);
        ++i;
    }

    PRINTMSG("Planning done.");
    res.error_code.value = manipulation_msgs::GraspPlanningErrorCode::SUCCESS;
    return true;
}
