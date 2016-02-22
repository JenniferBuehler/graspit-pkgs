#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Simple client for saving the current graspit world to file

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

#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <grasp_planning_graspit_msgs/SaveWorld.h>

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
    PRINTMSG("Usage: " << progName << " <filename> <as inventor: 'true' or 'false'>");
    PRINTMSG("If file is not saved as inventor, it will be saved as graspit XML file.");
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

    const std::string filenameArg(argv[1]);
    const std::string asInventorArg(argv[2]);

    PRINTMSG("Save world as file " << filenameArg << ", as inventor: " << asInventorArg);

    bool asInventorOpt = false;
    if (asInventorArg == trueStr)
    {
        asInventorOpt = true;
    }
    else if (asInventorArg != falseStr)
    {
        PRINTERROR("Must specify whether to save as inventor as '" << trueStr << "' or '" << falseStr << "'");
        printHelp(argv[0]);
        return 1;
    }

    // TODO parameterize this
    std::string saveWorldTopic = "graspit_save_world";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<grasp_planning_graspit_msgs::SaveWorld>(saveWorldTopic);

    grasp_planning_graspit_msgs::SaveWorld srv;
    srv.request.filename = filenameArg;
    srv.request.asInventor = asInventorOpt;

    if (!client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return 1;
    }

    if (!srv.response.success)
    {
        PRINTERROR("Could load save the world into file " << filenameArg);
        return 1;
    }

    PRINTMSG("Successfully saved world in " << filenameArg);
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
