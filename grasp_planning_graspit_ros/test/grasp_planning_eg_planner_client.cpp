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

#include <grasp_planning_graspit_ros/WriteToFile.h>
#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <moveit_msgs/GraspPlanning.h>

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
    PRINTMSG("Usage: " << progName << " <robot name> <object id> [<output-path>]");
    PRINTMSG("Where <robot name> is a string, <object id> is and id and <output-path> is the path where resulting Grasp.msg are written");
    PRINTMSG("Only robots/objects added to the database and loaded into the graspit world can be referenced.");
}

int run(int argc, char **argv)
{
    if (argc < 3)
    {
        printHelp(argv[0]);
        return 0;
    }

    std::string output_path;
    if (argc >= 4)
    {
        output_path=std::string(argv[3]);    
        if (!grasp_planning_graspit_ros::makeDirectoryIfNeeded(output_path))
        {
            PRINTERROR("Could not create directory "<<output_path);
            return 0;
        }
    }
        
    if (output_path.empty())
    {
        PRINTWARN("No output path configured, will print results on screen only.");
        printHelp(argv[0]);
    }

    const std::string robotArg(argv[1]);
    const std::string objectArg(argv[2]);

    PRINTMSG("Planning for robot ID=" << robotArg << " to grasp object ID=" << objectArg);

    // TODO parameterize this
    std::string egPlanningTopic = "graspit_eg_planning";

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<moveit_msgs::GraspPlanning>(egPlanningTopic);

    // Should use a client here to query the database for information about the
    // object type. For now, object type information is not used in the planning request,
    // as the service looks up the object type itself. So we can leave these on arbitrary values.
    object_recognition_msgs::ObjectType collModelType;
    collModelType.key =  "NotAvailabeYet";
    collModelType.db = "SimpleGraspItDatabase";

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

    moveit_msgs::CollisionObject collModel;
    collModel.header = modelPose.header;
    collModel.id = objectArg;
    collModel.type = collModelType;
    collModel.primitive_poses.push_back(modelPose.pose);
    PRINTWARN("Temporary hack: model pose for GraspPlanning service is "
      << "passed in first primitive pose. This should be handled by /tf "
      << "in future. See also issue #40.");

    moveit_msgs::GraspPlanning srv;
    srv.request.group_name = robotArg;
    srv.request.target = collModel;

    if (!client.call(srv))
    {
        PRINTERROR("Failed to call service");
        return 1;
    }

    if (srv.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        PRINTERROR("Could do the grasp planning. Error code " << srv.response.error_code.val);
        return 1;
    }

    PRINTMSG("Successfully finished grasp planning. Have " << srv.response.grasps.size() << " resulting grasps.");
    std::vector<moveit_msgs::Grasp>::iterator it;
    int i=1;
    for (it = srv.response.grasps.begin(); it != srv.response.grasps.end(); ++it)
    {
        if (!output_path.empty())
        {
            std::stringstream filenamePrefix;
            filenamePrefix << "Grasp_"<<i;
            ++i;
            if (!grasp_planning_graspit_ros::writeGraspMessage(*it, output_path, filenamePrefix.str()))
            {
                PRINTERROR("Could not save to file "<<filenamePrefix.str());
                continue;
            }
        }
        else
        {
            PRINTMSG(*it);
        }
    }
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
