#include <grasp_planning_graspit_ros/EigenGraspPlannerClient.h>
#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <grasp_planning_graspit_ros/WriteToFile.h>
#include <geometry_msgs/Pose.h>

#define DEFAULT_ADD_DB_TOPIC "graspit_add_to_db"
#define DEFAULT_LOAD_MODEL_TOPIC "graspit_add_to_db"
#define DEFAULT_SAVE_WORLD_TOPIC "graspit_save_world"
#define DEFAULT_EGPLANNING_TOPIC "graspit_eg_planning"


using grasp_planning_graspit_ros::EigenGraspPlannerClient;

EigenGraspPlannerClient::EigenGraspPlannerClient():
    initialized(false)
{
    init();
}
EigenGraspPlannerClient::~EigenGraspPlannerClient(){}
    
bool EigenGraspPlannerClient::isOK()
{
    return initialized && 
            eg_planner_client.exists() && eg_planner_client.isValid() &&
            add_to_db_client.exists() && add_to_db_client.isValid() &&
            load_model_client.exists() && load_model_client.isValid();
}

void EigenGraspPlannerClient::init()
{
    ros::NodeHandle priv("~");
    if (!priv.hasParam("add_to_db_service") ||
        !priv.hasParam("load_model_service") ||
        !priv.hasParam("save_world_service") ||
        !priv.hasParam("eg_planning_service"))
    {
        ROS_ERROR("Have to define the GraspIt! service types as ROS parameters. Won't be able to use EigenGraspPlannerClient.");
        return;
    }
    priv.param<std::string>("add_to_db_service", addToDBService, DEFAULT_ADD_DB_TOPIC);
    PRINTMSG("Using add_to_db_service: " << addToDBService);
    priv.param<std::string>("load_model_service", loadModelService, DEFAULT_LOAD_MODEL_TOPIC);
    PRINTMSG("Using load_model_service: " << loadModelService);
    priv.param<std::string>("save_world_service", saveWorldService, DEFAULT_SAVE_WORLD_TOPIC);
    PRINTMSG("Using save_world_service: " << saveWorldService);
    priv.param<std::string>("eg_planning_service", egPlanningService, DEFAULT_EGPLANNING_TOPIC);
    PRINTMSG("Using eg_planning_service: " << egPlanningService);
   
    add_to_db_client = node.serviceClient<grasp_planning_graspit_msgs::AddToDatabase>(addToDBService);
    load_model_client = node.serviceClient<grasp_planning_graspit_msgs::LoadDatabaseModel>(loadModelService);
    eg_planner_client = node.serviceClient<moveit_msgs::GraspPlanning>(egPlanningService);
    initialized = true;
}


int EigenGraspPlannerClient::addRobot(const std::string& modelName, const std::string filename, const std::vector<std::string>& jointNames)
{
    if (!isOK())
    {
        ROS_ERROR("EigenGraspPlannerClient not isOK() properly.");
        return -3;
    }
    PRINTMSG("Adding robot name " << modelName << " in file " << filename);
    grasp_planning_graspit_msgs::AddToDatabase srv;
    srv.request.filename = filename;
    srv.request.isRobot = true;
    srv.request.asGraspable = false;
    srv.request.modelName = modelName;
    srv.request.jointNames = jointNames;

    if (!add_to_db_client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return -1;
    }

    if (srv.response.returnCode != grasp_planning_graspit_msgs::AddToDatabase::Response::SUCCESS)
    {
        PRINTERROR("Could not add the robot to the database. Return code " << srv.response.returnCode);
        return -2;
    }

    PRINTMSG("Successfully added model to database, got model ID=" << srv.response.modelID);
    return srv.response.modelID;
}

int EigenGraspPlannerClient::addObject(const std::string& modelName, const std::string filename, bool objectGraspable)
{
    if (!isOK())
    {
        ROS_ERROR("EigenGraspPlannerClient not isOK() properly.");
        return -3;
    }
    PRINTMSG("Adding object name " << modelName << " in file " << filename);
    grasp_planning_graspit_msgs::AddToDatabase srv;
    srv.request.filename = filename;
    srv.request.isRobot = false;
    srv.request.asGraspable = objectGraspable;
    srv.request.modelName = modelName;

    if (!add_to_db_client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return -1;
    }

    if (srv.response.returnCode != grasp_planning_graspit_msgs::AddToDatabase::Response::SUCCESS)
    {
        PRINTERROR("Could not add the object to the database. Return code " << srv.response.returnCode);
        return -2;
    }

    PRINTMSG("Successfully added object to database, got model ID=" << srv.response.modelID);
    return srv.response.modelID;
} 

int EigenGraspPlannerClient::loadModel(const int modelID, bool clearWorld, const geometry_msgs::Pose& modelPose)
{
    if (!isOK())
    {
        ROS_ERROR("EigenGraspPlannerClient not isOK() properly.");
        return -3;
    }
    //PRINTMSG("Loading model " << modelID << " to graspit world, at pose "<<modelPose<<std::endl<<"Clear others: " << clearWorld);

    grasp_planning_graspit_msgs::LoadDatabaseModel srv;
    srv.request.model_id = modelID;
    srv.request.model_pose = modelPose;
    srv.request.clear_other_models = clearWorld;

    if (!load_model_client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return -1;
    }

    if (srv.response.result != grasp_planning_graspit_msgs::LoadDatabaseModel::Response::LOAD_SUCCESS)
    {
        PRINTERROR("Could load model ID=" << modelID);
        return -2;
    }

    PRINTMSG("Successfully loaded model ID=" << modelID);
    return 0;
}


int EigenGraspPlannerClient::plan(const std::string robotModelName, const int objectID,
    const geometry_msgs::Pose * newObjectPose,
    const std::string& resultsOutputDirectory,
    std::vector<moveit_msgs::Grasp>& results)
{
    if (!isOK())
    {
        ROS_ERROR("EigenGraspPlannerClient not isOK() properly.");
        return -4;
    }
    if (resultsOutputDirectory.empty())
    {
        PRINTMSG("No output path configured, results will be returned in vector only");
    }
    else if (!makeDirectoryIfNeeded(resultsOutputDirectory))
    {
        PRINTERROR("Could not create directory "<<resultsOutputDirectory);
        return -1;
    }

    PRINTMSG("Planning for robot ID=" << robotModelName << " to grasp object ID=" << objectID);

    // Should use a client here to query the database for information about the
    // object type. For now, object type information is not used in the planning request,
    // as the service looks up the object type itself. So we can leave these on arbitrary values.
    object_recognition_msgs::ObjectType dbModelType;
    dbModelType.key =  "NotAvailabeYet";
    dbModelType.db = "SimpleGraspItDatabase";

    // Here we can set a different pose to put the object at in the current
    // graspit world. If this reference frame is "0",
    // it uses the object's current pose in the GraspIt! world. If it is "1",
    // we will use the pose specified here.
    geometry_msgs::PoseStamped modelPose;
    if (newObjectPose != NULL)
    {
        modelPose.pose = *newObjectPose;
        modelPose.header.frame_id = "1";
    }
    else
    {
        modelPose.header.frame_id = "0";
    }

    moveit_msgs::CollisionObject obj;
    obj.header = modelPose.header;
    obj.id = std::to_string(objectID);
    obj.type = dbModelType;
    obj.primitive_poses.push_back(modelPose.pose);
    PRINTWARN("Temporary hack: model pose for GraspPlanning service is "
      << "passed in first primitive pose. This should be handled by /tf "
      << "in future. See also issue #40.");

    moveit_msgs::GraspPlanning srv;
    srv.request.group_name = robotModelName;
    srv.request.target = obj;

    if (!eg_planner_client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return -2;
    }

    if (srv.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        PRINTERROR("Could do the grasp planning. Error code " << srv.response.error_code.val);
        return -3;
    }

    PRINTMSG("Successfully finished grasp planning. Have " << srv.response.grasps.size() << " resulting grasps.");
    std::vector<moveit_msgs::Grasp>::iterator it;
    int i=1;
    for (it = srv.response.grasps.begin(); it != srv.response.grasps.end(); ++it)
    {
        if (!resultsOutputDirectory.empty())
        {
            std::stringstream filenamePrefix;
            filenamePrefix << "Grasp_" << i << ".msg";
            ++i;
            PRINTMSG("Saving grasp " << i << " to file (dir "
              << resultsOutputDirectory << ", prefix " << filenamePrefix.str() << ")");
            if (!writeGraspMessage(*it, resultsOutputDirectory, filenamePrefix.str()))
            {
                PRINTERROR("Could not save grasp to file "<<filenamePrefix.str());
                continue;
            }
        }
        results.push_back(*it);
    }
    return 0;
}

