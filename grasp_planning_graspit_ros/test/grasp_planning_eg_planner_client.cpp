#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Simple grasp planning client building only a very basic GraspPlanning.srv

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
#endif  // DOXYGEN_SHOULD_SKIP_THIS

#include <string>
#include <vector>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>

#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <manipulation_msgs/GraspPlanning.h>

#include <eigen_conversions/eigen_msg.h>

/**
 * Helper method to print the trace in case of a SIG* event
 */
void print_trace(void)
{
    void *array[10];
    size_t size;
    char **strings;
    size_t i;

    size = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    printf("Obtained %zd stack frames.\n", size);

    for (i = 0; i < size; i++)
        printf("%s\n", strings[i]);

    free(strings);
}


void handler(int sig)
{
    print_trace();
    ros::shutdown();
    exit(1);
}

void printHelp(const char * progName)
{
    PRINTMSG("Usage: " << progName << " <robot name> <object id>");
    PRINTMSG("Only objects added to the database and loaded into the graspit world can be referenced.");
}

int run(int argc, char **argv)
{
    if (argc < 3)
    {
        printHelp(argv[0]);
        return 0;
    }

    const std::string robotArg(argv[1]);
    const std::string objectArg(argv[2]);

    PRINTMSG("Planning for robot ID=" << robotArg << " to grasp object ID=" << objectArg);

    // TODO parameterize this
    std::string egPlanningTopic = "graspit_eg_planning";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<manipulation_msgs::GraspPlanning>(egPlanningTopic);

    // Should use a client here to query the database for information about the
    // object type. For now, object type information is not used in the planning request,
    // as the service looks up the object type itself. So we can leave these on arbitrary values.
    object_recognition_msgs::ObjectType dbModelType;
    dbModelType.key =  "NotAvailabeYet";
    dbModelType.db = "SimpleGraspItDatabase";

    // Here we can set a different pose to put the object at in the current
    // graspit world. If this the reference frame is "0",
    // it uses the object's current pose in the world. If it is "1",
    // we will use the pose specified in the following field.
    geometry_msgs::PoseStamped modelPose;
    modelPose.header.frame_id = "0";

    /*    modelPose.header.frame_id="1";
          modelPose.pose.orientation.w=1;
          modelPose.pose.position.x=100;
     */

    household_objects_database_msgs::DatabaseModelPose dbModel;
    dbModel.model_id = atoi(objectArg.c_str());  // TODO move away from atoi at some stage
    dbModel.type = dbModelType;
    dbModel.pose = modelPose;
    dbModel.confidence = 1;
    dbModel.detector_name = "manual_detection";


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
    srv.request.arm_name = robotArg;
    srv.request.target = obj;
    srv.request.collision_object_name = obj.collision_name;
    // srv.request.collision_support_surface_name = will not provide this here
    // srv.request.grasps_to_evaluate = no grasps to evaluate with this client
    // srv.request.movable_obstacles = this is not supported by this client

    if (!client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return 1;
    }

    if (srv.response.error_code.value != manipulation_msgs::GraspPlanningErrorCode::SUCCESS)
    {
        PRINTERROR("Could do the grasp planning. Error code " << srv.response.error_code.value);
        return 1;
    }

    PRINTMSG("Successfully finished grasp planning. Have "<<srv.response.grasps.size()<<" resulting grasps.");
/*    std::vector<manipulation_msgs::Grasp>::iterator it;
    for (it = srv.response.grasps.begin(); it != srv.response.grasps.end(); ++it)
    {
        PRINTMSG(*it);
    }
*/
    return 0;
}


int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    ros::init(argc, argv, "database_test", ros::init_options::AnonymousName);

    bool useRosLogging = true;
    if (useRosLogging)
    {
        PRINT_INIT_ROS();
    }
    else
    {
        PRINT_INIT_STD();
    }

    int ret = run(argc, argv);
    return ret;
}
