#include <grasp_planning_graspit_ros/EigenGraspPlannerClient.h>
#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <fstream>
#include <boost/filesystem.hpp>
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
    eg_planner_client = node.serviceClient<manipulation_msgs::GraspPlanning>(egPlanningService);
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
    std::vector<manipulation_msgs::Grasp>& results)
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

    household_objects_database_msgs::DatabaseModelPose dbModel;
    dbModel.model_id = objectID;
    dbModel.type = dbModelType;
    dbModel.pose = modelPose;
    dbModel.confidence = 1;
    dbModel.detector_name = "graspit_eigengrasp_planner_client";

    manipulation_msgs::GraspableObject obj;
    // the reference frame could be one that is relative to all fields (e.g. cluster and
    // all potential models). However at the moment, the graspit planner only supports
    // the global frame (the graspit origin). No tf transforms are considered in the
    // GraspIt planner service yet.
    obj.reference_frame_id = dbModel.pose.header.frame_id;
    obj.potential_models.push_back(dbModel);
    // obj.cluster = we will not provide a point cloud
    // obj.region = and not the SceneRegion along with it either.
    // obj.collision_name = could think about whether providing this as parameter too

    manipulation_msgs::GraspPlanning srv;
    srv.request.arm_name = robotModelName;
    srv.request.target = obj;
    srv.request.collision_object_name = obj.collision_name;
    // srv.request.collision_support_surface_name = will not provide this here
    // srv.request.grasps_to_evaluate = no grasps to evaluate with this client
    // srv.request.movable_obstacles = this is not supported by this client

    if (!eg_planner_client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return -2;
    }

    if (srv.response.error_code.value != manipulation_msgs::GraspPlanningErrorCode::SUCCESS)
    {
        PRINTERROR("Could do the grasp planning. Error code " << srv.response.error_code.value);
        return -3;
    }

    PRINTMSG("Successfully finished grasp planning. Have " << srv.response.grasps.size() << " resulting grasps.");
    std::vector<manipulation_msgs::Grasp>::iterator it;
    int i=1;
    for (it = srv.response.grasps.begin(); it != srv.response.grasps.end(); ++it)
    {
        if (!resultsOutputDirectory.empty())
        {
            std::stringstream filename;
            filename<<resultsOutputDirectory<<"/Grasp_"<<i<<".msg";
            std::stringstream filename_txt;
            filename_txt<<resultsOutputDirectory<<"/Grasp_"<<i<<"_string.msg";
            ++i;
            ROS_INFO_STREAM("Saving grasp "<<i<<" to file "<<filename.str());
            if (!saveToFile(*it, filename.str(), true))
            {
                PRINTERROR("Could not save grasp to file "<<filename.str());
                continue;
            }
            ROS_INFO_STREAM("Saving text form of grasp "<<i<<" to file "<<filename_txt.str());
            saveToFile(*it, filename_txt.str(), false);
        }
        results.push_back(*it);
    }
    return 0;
}

bool EigenGraspPlannerClient::saveToFile(const manipulation_msgs::Grasp& msg, const std::string& filename, bool asBinary)
{

    std::ios_base::openmode mode;
    if (asBinary) mode = std::ios::out | std::ios::binary;
    else mode = std::ios::out;

    std::ofstream ofs(filename.c_str(), mode);

    if (!ofs.is_open())
    {
        ROS_ERROR("File %s cannot be opened.", filename.c_str());
        return false;
    }

    if (asBinary)
    {
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
        ros::serialization::OStream ostream(obuffer.get(), serial_size);
        ros::serialization::serialize(ostream, msg);
        ofs.write((char*) obuffer.get(), serial_size);
    }
    else
    {
        ofs<<msg; 
    }
    ofs.close();
    return true;
}


bool EigenGraspPlannerClient::makeDirectoryIfNeeded(const std::string& dPath)
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
