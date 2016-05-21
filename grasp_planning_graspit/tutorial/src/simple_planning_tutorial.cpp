#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Tutorial for running the graspit planner on a given graspit world

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
#endif   // DOXYGEN_SHOULD_SKIP_THIS

#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <string>
#include <vector>

/**
 * \brief Tutorial on how to load a world and run the Eigengrasp planner.
 */
int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <world-filename> <output-directory>" << std::endl;
        return 1;
    }

    std::string worldFilename = argv[1];
    std::string outputDirectory = argv[2];

    if (worldFilename.empty())
    {
        std::cerr << "You have to specify a world" << std::endl;
        return 1;
    }


    // Create the graspit world manager.
    SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());

    // Load the graspit world
    graspitMgr->loadWorld(worldFilename);

    // if the output directory does not exist yet, create it?
    bool createDir = true;

    // in case one wants to view the initial world before planning, save it:
    graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld.xml", createDir);
    graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld.iv", createDir);

    // Create the planner which accesses the graspit world.
    std::string name = "EigenGraspPlanner1";
    SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner(name, graspitMgr));

    // Number of iterations for the planning algorithm
    int maxPlanningSteps = 70000;
    // Number of times to repeat the planning process
    int repeatPlanning = 1;
    
    // Maximum number of planning results to keep (of each planning repeat)
    int keepMaxPlanningResults = 3;
    // Finalize each planning result with an "auto-grasp" to ensure there really are
    // contacts between fingers and objects (sometimes, the grasp result is just very
    // close to the object, but not really touching it).
    bool finishWithAutograsp = false;

    // By default, the last robot loaded and the last object loaded are to be used as the hand and the
    // object to grasp for the planning process. You can use the other EigenGraspPlanner::plan()
    // method with more parameters to change this.
    if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, finishWithAutograsp))
    {
        std::cerr << "Error doing the planning." << std::endl;
        return 1;
    }

    // Now, save the results as world files.

    // specify where you want to save them:
    std::string resultsWorldDirectory = outputDirectory + "/worlds";
    // each result file will start with this prefix:
    std::string filenamePrefix = "world";
    // specify to save as inventor files:
    bool saveInventor = true;
    // specify to save as graspit world files:
    bool saveGraspit = true;


    // Now, save the results.
    planner->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, saveGraspit, saveInventor, createDir);

    // Iterate through all results and print information about the grasps:

    std::vector<GraspIt::EigenGraspResult> allGrasps;
    planner->getResults(allGrasps);

    std::cout << "Grasp results:" << std::endl;
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        std::cout << *it << std::endl;;
    }

    return 0;
}
