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

#include <grasp_planning_graspit/GraspItSceneManagerNoGui.h>
#include <grasp_planning_graspit/LogBinding.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspPlannerNoQt.h>
#include <grasp_planning_graspit/EigenGraspResult.h>

#include <string>
#include <set>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

// if this is defined, the EigenGraspPlannerNoQt implementation
// is used. If not defined, EigenGraspPlannern implementation is used.
// #define USE_EIGENGRASP_NOQT


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


boost::program_options::options_description getOptions()
{
    // Declare the supported options.
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("dir", boost::program_options::value<std::string>(), "set output directory for resulting files")
    ("wld", boost::program_options::value<std::string>(), "filename for the world file")
    ("rob", boost::program_options::value<std::string>(), "filename for the robot file -- OPTIONAL to parameter wld!")
    ("obj", boost::program_options::value<std::string>(), "filename for the object file -- OPTIONAL to parameter wld!");
    return desc;
}

boost::program_options::variables_map loadParams(int argc, char ** argv)
{
    boost::program_options::options_description optDesc = getOptions();
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, optDesc), vm);
    boost::program_options::notify(vm);
    return vm;
}

bool loadParams(int argc, char ** argv, std::string& worldFilename, std::string& robotFilename,
                std::string& objectFilename, std::string& outputDirectory)
{
    worldFilename.clear();
    robotFilename.clear();
    objectFilename.clear();
    outputDirectory.clear();

    boost::program_options::variables_map vm;
    try
    {
        vm = loadParams(argc, argv);
    }
    catch (std::exception const& e)
    {
        PRINTERROR("Exception caught: " << e.what());
        return false;
    }
    catch (...)
    {
        PRINTERROR("Exception caught");
        return false;
    }

    boost::program_options::options_description desc = getOptions();
    // desc=getOptions();

    if (vm.count("help"))
    {
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("dir") < 1)
    {
        PRINTERROR("Must specify an output directory");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("wld") && (vm.count("rob") || vm.count("obj")))
    {
        PRINTERROR("Cannot specify a world and a robot and/or object at the same time.");
        PRINTMSG(desc);
        return false;
    }

    if (!vm.count("wld") && !vm.count("rob"))
    {
        PRINTERROR("Have to specify either a robot or a world.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("rob") != vm.count("obj"))
    {
        PRINTERROR("If you specify a robot, you also have to specify an object, and vice versa.");
        PRINTMSG(desc);
        return false;
    }


    if (vm.count("rob") > 1)
    {
        PRINTERROR("You can only specify one robot at this stage.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("obj") > 1)
    {
        PRINTERROR("You can only specify one object at this stage.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("obj") != vm.count("rob"))
    {
        PRINTERROR("If you specify a robot, you should also specify an object.");
        PRINTMSG(desc);
        return false;
    }

    if (vm.count("wld"))
    {
        worldFilename = vm["wld"].as<std::string>();
        PRINTMSG("World file is " << worldFilename);
    }
    if (vm.count("rob"))
    {
        robotFilename = vm["rob"].as<std::string>();
        PRINTMSG("Robot file is " << robotFilename);
    }
    if (vm.count("obj"))
    {
        objectFilename = vm["obj"].as<std::string>();
        PRINTMSG("Object file is " << objectFilename);
    }
    if (vm.count("dir"))
    {
        outputDirectory = vm["dir"].as<std::string>();
        PRINTMSG("Output dir is " << outputDirectory);
    }

    return true;
}


int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    PRINT_INIT_STD();

    std::string worldFilename;
    std::string robotFilename;
    std::string objectFilename;
    std::string outputDirectory;

    if (!loadParams(argc, argv, worldFilename, robotFilename, objectFilename, outputDirectory))
    {
        return 1;
    }

    PRINTMSG("Creating planner");

    std::string name = "EigenGraspPlanner1";  // TODO make parameter
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerNoGui());

#ifdef USE_EIGENGRASP_NOQT
    SHARED_PTR<GraspIt::EigenGraspPlannerNoQt> p(new GraspIt::EigenGraspPlannerNoQt(name, graspitMgr));
#else
    SHARED_PTR<GraspIt::EigenGraspPlanner> p(new GraspIt::EigenGraspPlanner(name, graspitMgr));
#endif

    if (!worldFilename.empty())
    {
        PRINTMSG("Loading world");
        graspitMgr->loadWorld(worldFilename);
    }
    else
    {
        // TODO add an option to set the transforms.
        // For now, they're put in the origin. For the planning, this should not really matter...
        GraspIt::EigenTransform robotTransform;
        GraspIt::EigenTransform objectTransform;
        robotTransform.setIdentity();
        objectTransform.setIdentity();
        // objectTransform.translate(Eigen::Vector3d(100,0,0));
        std::string robotName("Robot1");  // TODO parameterize
        std::string objectName("Object1");
        if ((graspitMgr->loadRobot(robotFilename, robotName, robotTransform) != 0) ||
                (graspitMgr->loadObject(objectFilename, objectName, true, objectTransform)))
        {
            PRINTERROR("Could not load robot or object");
            return 1;
        }

        // in case one wants to view the initial world before planning, save it:
        graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld.xml");
        graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld.iv");
    }

    // now save the world again as inventor file, to test
    // p->saveIVWorld("test.iv");

    int maxPlanningSteps = 50000;
    int repeatPlanning = 1;
    int keepMaxPlanningResults = 3;
    bool finishWithAutograsp = false;
    p->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, finishWithAutograsp);

    PRINTMSG("Saving results as world files");

    bool createDir = true;
    bool saveIV = true;
    bool saveWorld = true;

    std::string resultsWorldDirectory = outputDirectory;
    std::string filenamePrefix = "world";
    p->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, saveWorld, saveIV, createDir);

    std::vector<GraspIt::EigenGraspResult> allGrasps;
    p->getResults(allGrasps);

    PRINTMSG("Grasp results:");
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        PRINTMSG(*it);
    }

    PRINTMSG("Quitting program.");
    return 1;
}
