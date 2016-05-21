#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Main method for running the graspit planner on a given graspit world (or robot and object)

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
#include <vector>

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
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

/**
 * Accepts following ROS parameters within the nodes private namespace:
 *
 * - "use_world_file": set to true if a world file is to be loaded.
 *     Alternatively, a robot and object may be loaded separately.
 * - "graspit_world_file": the graspit world file. Use EITHER this OR specify a robot and object.
 * - "graspit_robot_file": the graspit robot file. Use EITHER this OR specify a world.
 *      if you specify a robot, you also have to specify an object.
 * - "graspit_object_file: the graspit object file.
 *      if you specify an object, you also have to specify a robot.
 * - "graspit_planning_algorithm": The planning algorithm to use. At this stage,
 *     only simulated annealing ("SimAnn") is supported.
 * - "max_planning_steps": the maximum planning steps to use for the Eigengrasp planner.
 * - "results_output_directory": the directory where results should be saved
 * - "save_result_files_inventor": a boolean flag specifying whether to save
 *    the results as inventor files in the folder specified in "results_output_directory".
 * - "save_result_files_graspit": a boolean flag specifying whether to save
 *    the results as graspit world files in the folder specified in "results_output_directory".
 */
int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    ros::init(argc, argv, "grasp_planning_graspit", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");


    bool useRosLogging = true;
    if (useRosLogging)
    {
        PRINT_INIT_ROS();
    }
    else
    {
        PRINT_INIT_STD();
    }


    ROS_INFO_STREAM("Reading parameters from namespace " << priv.getNamespace());

    std::string outputDirectory;
    if (!priv.hasParam("results_output_directory"))
    {
        PRINTERROR("Could not read parameter results_output_directory");
        return 1;
    }
    priv.param<std::string>("results_output_directory", outputDirectory, outputDirectory);
    PRINTMSG("Results output directory: " << outputDirectory);

    bool useWorld = false;
    if (!priv.hasParam("use_world_file"))
    {
        PRINTERROR("Could not read parameter use_world_file");
        return 1;
    }
    priv.param<bool>("use_world_file", useWorld, useWorld);

    std::string worldFilename;
    std::string robotFilename;
    std::string objectFilename;

    if (useWorld)
    {
        PRINTMSG("Using world file");
        if (!priv.hasParam("graspit_world_file"))
        {
            PRINTERROR("Could not read parameter graspit_world_file");
            return 1;
        }
        priv.param<std::string>("graspit_world_file", worldFilename, worldFilename);
        PRINTMSG("Using world file " << worldFilename);
    }
    else
    {
        PRINTMSG("Using robot and object file");
        if (!priv.hasParam("graspit_robot_file"))
        {
            PRINTERROR("Could not read parameter graspit_robot_file");
            return 1;
        }
        priv.param<std::string>("graspit_robot_file", robotFilename, robotFilename);
        PRINTMSG("Using robot file " << robotFilename);

        if (!priv.hasParam("graspit_object_file"))
        {
            PRINTERROR("Could not read parameter graspit_object_file");
            return 1;
        }
        priv.param<std::string>("graspit_object_file", objectFilename, objectFilename);
    }

    int maxPlanningSteps;
    priv.param<int>("max_planning_steps", maxPlanningSteps, 50000);
    PRINTMSG("Using max number of planning steps: " << maxPlanningSteps);

    int repeatPlanning;
    priv.param<int>("num_repeat_planning", repeatPlanning, 1);
    PRINTMSG("Repeating planning: " << repeatPlanning);


    int numKeepResults;
    priv.param<int>("default_num_keep_results", numKeepResults, 5);
    PRINTMSG("Using default number of results kept: " << numKeepResults);
    
    bool finishWithAutograsp;
    priv.param<bool>("default_finish_with_autograsp", finishWithAutograsp, false);
    PRINTMSG("Finish with auto-grasp by default: " << finishWithAutograsp);

    bool saveResultFilesInventor;
    priv.param<bool>("save_result_files_inventor", saveResultFilesInventor, true);
    PRINTMSG("Save result files inventor: " << saveResultFilesInventor);

    bool saveResultFilesGraspit;
    priv.param<bool>("save_result_files_graspit", saveResultFilesGraspit, true);
    PRINTMSG("Save result files graspit: " << saveResultFilesGraspit);

    PRINTMSG("Creating planner");
    std::string name = "EigenGraspPlanner1";  // TODO make parameter
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

    SHARED_PTR<GraspIt::EigenGraspPlanner> p(new GraspIt::EigenGraspPlanner(name, graspitMgr));

    if (!worldFilename.empty())
    {
        PRINTMSG("Loading world");
        graspitMgr->loadWorld(worldFilename);
    }
    else
    {
        // TODO add an option to specify a transform for the object.
        // For now, they're put in the origin. For the planning, this should not really matter...
        GraspIt::EigenTransform robotTransform;
        GraspIt::EigenTransform objectTransform;
        robotTransform.setIdentity();
        objectTransform.setIdentity();
        // objectTransform.translate(Eigen::Vector3d(100,0,0));
        std::string robotName("Robot1");  // TODO parameterize
        std::string objectName("Object1");  // TODO parameterize
        if ((graspitMgr->loadRobot(robotFilename, robotName, robotTransform) != 0) ||
                (graspitMgr->loadObject(objectFilename, objectName, true, objectTransform) != 0))
        {
            PRINTERROR("Could not load robot or object");
            return 0;
        }

        // in case one wants to view the initial world before planning, save it:
        graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld.xml", true);
        graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld.iv", true);
    }

    PRINTMSG("Now planning...");
    if (!p->plan(maxPlanningSteps, repeatPlanning, numKeepResults, finishWithAutograsp))
    {
        PRINTERROR("Could not plan.");
        return 0;
    }

    PRINTMSG("Saving results as world files");

    std::string resultsWorldDirectory = outputDirectory + "/worlds";
    std::string filenamePrefix = "world";
    p->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, true, true);

    std::vector<GraspIt::EigenGraspResult> allGrasps;
    p->getResults(allGrasps);

    PRINTMSG("Grasp results:");
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        PRINTMSG(*it);
    }


    PRINTMSG("Quitting program.");
    return 0;
}
