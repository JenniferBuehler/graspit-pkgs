#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   ROS service for grasp planning with the GraspIt EigenGrasp planner interface.

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
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>

#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <grasp_planning_graspit_ros/GraspItServices.h>


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

int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    ros::init(argc, argv, "grasp_planning_graspit_service", ros::init_options::AnonymousName);

    bool useRosLogging = true;
    if (useRosLogging)
    {
        PRINT_INIT_ROS();
    }
    else
    {
        PRINT_INIT_STD();
    }

    GraspIt::GraspItServices s;
    s.start();

    ros::spin();

    PRINTMSG("Quitting services.");
}
