#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Sample client for loading a model from the graspit database to the world.

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
#include <set>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <grasp_planning_graspit_msgs/LoadDatabaseModel.h>

#include <grasp_planning_graspit/GraspItTypes.h>

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
    PRINTMSG("Usage: " << progName << " <database model id> <clear other models: 'true' or 'false'> [<x> <y> <z>] [<qw> <qx> <qy> <qz>]");
}

int run(int argc, char **argv)
{
    if (argc < 3)
    {
        printHelp(argv[0]);
        return 0;
    }

    const std::string falseStr("false");
    const std::string trueStr("true");

    const std::string modelIDArg(argv[1]);
    const std::string clearOthersArg(argv[2]);

    PRINTMSG("Loading model " << modelIDArg << " to graspit world. Clear others: " << clearOthersArg);

    bool clearOthersOpt = false;
    if (clearOthersArg == trueStr)
    {
        clearOthersOpt = true;
    }
    else if (clearOthersArg != falseStr)
    {
        PRINTERROR("Must specify whether to clear other bodies " << trueStr << " or " << falseStr);
        printHelp(argv[0]);
        return 1;
    }

    GraspIt::EigenTransform transform;
    transform.setIdentity();

    if (argc > 3)
    {
        if ((argc != 6) && (argc != 10))
        {
            PRINTERROR("If you specify more than 2 arguments, you must specify either a) a translation or b) a complete transform");
            printHelp(argv[0]);
            return 1;
        }

        // specified transform into world
        if (argc >= 6)
        {
            // specified only translation
            // TODO get rid of using atof at some stage
            float x = atof(argv[3]);
            float y = atof(argv[4]);
            float z = atof(argv[5]);
            // PRINTMSG("Translation: "<<x<<", "<<y<<", "<<z);
            transform = transform.translate(Eigen::Vector3d(x, y, z));
        }
        if (argc == 10)
        {
            // specified complete transform
            // TODO get rid of using atof at some stage
            float qw = atof(argv[6]);
            float qx = atof(argv[7]);
            float qy = atof(argv[8]);
            float qz = atof(argv[9]);
            // PRINTMSG("Quaternion: "<<qw<<", "<<qx<<", "<<qy<<", "<<qz);
            transform = transform.rotate(Eigen::Quaterniond(qw, qx, qy, qz));
        }
    }


    geometry_msgs::Pose modelPose;
    tf::poseEigenToMsg(transform, modelPose);

    // TODO parameterize this
    std::string loadModelTopic = "graspit_load_model";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<grasp_planning_graspit_msgs::LoadDatabaseModel>(loadModelTopic);

    // TODO get rid of using atof at some stage
    int modelID = atof(modelIDArg.c_str());

    grasp_planning_graspit_msgs::LoadDatabaseModel srv;
    srv.request.model_id = modelID;
    srv.request.model_pose = modelPose;
    srv.request.clear_other_models = clearOthersOpt;

    if (!client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return 1;
    }

    if (srv.response.result != grasp_planning_graspit_msgs::LoadDatabaseModel::Response::LOAD_SUCCESS)
    {
        PRINTERROR("Could load model ID=" << modelIDArg);
        return 1;
    }

    PRINTMSG("Successfully loaded model ID=" << modelID);
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
    // PRINTMSG("Result of run(): "<<ret);
    return ret;
}
