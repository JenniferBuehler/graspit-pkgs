#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Simple client for adding a sample model to the graspit database

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

#include <grasp_planning_graspit_msgs/AddToDatabase.h>

#include <grasp_planning_graspit_ros/LogBindingROS.h>


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
    PRINTMSG("Usage: " << progName << " <model name> <model file> <as robot: 'true' or 'false'> [<for objects: as graspable, 'true' or 'false'>]");
}

int run(int argc, char **argv)
{
    if (argc < 4)
    {
        printHelp(argv[0]);
        return 0;
    }

    const std::string falseStr("false");
    const std::string trueStr("true");

    const std::string modelName(argv[1]);
    const std::string filename(argv[2]);
    const std::string asRobotArg(argv[3]);

    PRINTMSG("Adding model name " << modelName << " in file " << filename << ". As robot: " << asRobotArg);

    bool asRobotOpt = false;
    if (asRobotArg == trueStr)
    {
        asRobotOpt = true;
    }
    else if (asRobotArg != falseStr)
    {
        PRINTERROR("Must specify whether to load as robot with " << trueStr << " or " << falseStr);
        printHelp(argv[0]);
        return 1;
    }


    int jnIdx = 4;  // index in argv where joint names start
    bool asGraspableOpt = false;
    if (!asRobotOpt)
    {
        if (argc < 5)
        {
            PRINTERROR("If loading an object, you must specify whether it is to be loaded as graspable");
            printHelp(argv[0]);
            return 1;
        }
        const std::string asGraspableArg(argv[4]);
        jnIdx = 5;
        if (asGraspableArg == trueStr)
        {
            PRINTMSG("Loading object as graspable");
            asGraspableOpt = true;
        }
        else if (asGraspableArg != falseStr)
        {
            PRINTERROR("Must specify whether to load as graspable object with " << trueStr << " or " << falseStr);
            printHelp(argv[0]);
            return 1;
        }
    }

    if (asRobotOpt && (jnIdx >= argc))
    {
        PRINTERROR("Must specify at least one joint name for a robot!");
        printHelp(argv[0]);
        return 1;
    }

    std::vector<std::string> jointNames;
    for (int i = jnIdx; i < argc; ++i)
    {
        jointNames.push_back(std::string(argv[i]));
    }


    // TODO parameterize this
    std::string addToDBTopic = "graspit_add_to_database";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<grasp_planning_graspit_msgs::AddToDatabase>(addToDBTopic);

    grasp_planning_graspit_msgs::AddToDatabase srv;
    srv.request.filename = filename;
    srv.request.isRobot = (asRobotOpt == 1);
    srv.request.asGraspable = (asGraspableOpt == 1);
    srv.request.modelName = modelName;
    srv.request.jointNames = jointNames;

    if (!client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return 1;
    }

    if (srv.response.returnCode != grasp_planning_graspit_msgs::AddToDatabase::Response::SUCCESS)
    {
        PRINTERROR("Could not add the robot to the database. Return code " << srv.response.returnCode);
        return 1;
    }

    PRINTMSG("Successfully added model to database, got model ID=" << srv.response.modelID);
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
