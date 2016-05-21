#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Test file for running the simple graspit database

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

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/GraspItSimpleDBManager.h>
#include <grasp_planning_graspit/LogBinding.h>


/**
 * Helper method to print the trace in case of a SIG event
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
    exit(1);
}


int run(int argc, char **argv)
{
    if (argc < 4)
    {
        PRINTMSG("Usage: " << argv[0] << " <outputDir> <robotFile> <objectFile>");
        return 0;
    }

    std::string outputDirectory(argv[1]);
    std::string robotFilename(argv[2]);
    std::string objectFilename(argv[3]);

    PRINTMSG("Creating database");
    std::string name = "Database1";
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

    SHARED_PTR<GraspIt::GraspItSimpleDBManager> mgr(new GraspIt::GraspItSimpleDBManager(name, graspitMgr));

    std::string robotName("Robot1");
    std::string objectName("Object1");

    PRINTMSG("Now loading robot");

    int robotID = -1;
    int objectID = -1;

    // here we'd have to put the robot joint names, we'll leave this empty for this test fle
    std::vector<std::string> jointNames;
    if ((robotID = mgr->loadRobotToDatabase(robotFilename, robotName, jointNames)) < 0)
    {
        PRINTERROR("Could not load robot");
        return 0;
    }

    PRINTMSG("Now loading object");

    if ((objectID = mgr->loadObjectToDatabase(objectFilename, objectName, true)) < 0)
    {
        PRINTERROR("Could not load object");
        return 0;
    }

    // Now objects should be in the database only, but not in the world.
    // Re-add them to test if the world still workds afterwards:
    PRINTMSG("Now loading robot to world");
    if (mgr->loadToWorld(robotID, GraspIt::EigenTransform::Identity()) != 0)
    {
        PRINTERROR("Could not add the robot to the graspit world");
        return 0;
    }

    PRINTMSG("Now loading object to world");
    if (mgr->loadToWorld(objectID, GraspIt::EigenTransform::Identity()) != 0)
    {
        PRINTERROR("Could not add the object to the graspit world");
        return 0;
    }

    PRINTMSG("Saving world files");

    // test to see if it worked: save as world
    bool createDir = true;  // if true, the directory will be created, if it doesn't exist.
    graspitMgr->saveGraspItWorld(outputDirectory + "/dbtest/world.xml", createDir);
    graspitMgr->saveInventorWorld(outputDirectory + "/dbtest/world.iv", createDir);

    PRINTMSG("Quitting program.");
    return 0;
}

int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    PRINT_INIT_STD();

    int ret = run(argc, argv);
    PRINTMSG("Result of run(): " << ret);
    return 0;
}
