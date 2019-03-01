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
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>

#include <grasp_planning_graspit_ros/WriteToFile.h>
#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <moveit_msgs/Grasp.h>

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

  if (argc < 2)
  {
    PRINTERROR("Require a directory as command line argument");
    return 0;
  }

  std::string output_path = std::string(argv[1]);    
  if (!grasp_planning_graspit_ros::makeDirectoryIfNeeded(output_path))
  {
      PRINTERROR("Could not create directory "<<output_path);
      return 0;
  }
  
  trajectory_msgs::JointTrajectory ttemp;
  ttemp.joint_names.push_back("jaco_finger_joint_0");
  ttemp.joint_names.push_back("jaco_finger_joint_2");
  ttemp.joint_names.push_back("jaco_finger_joint_4");
  trajectory_msgs::JointTrajectoryPoint jpre;
  jpre.positions.resize(3, 0.0045);
  trajectory_msgs::JointTrajectoryPoint jpost;
  jpost.positions.resize(3, -0.432062);

  moveit_msgs::Grasp g1;
  g1.id = "jaco_robot_small_cube_0";
  g1.pre_grasp_posture = ttemp;
  g1.pre_grasp_posture.points.push_back(jpre);
  g1.grasp_posture = ttemp;
  g1.grasp_posture.points.push_back(jpost);
  g1.grasp_pose.pose.position.x =  -0.0402668;
  g1.grasp_pose.pose.position.y =  -0.0304912;
  g1.grasp_pose.pose.position.z =  0.185728;
  g1.grasp_pose.pose.orientation.x =  -0.36809;
  g1.grasp_pose.pose.orientation.y =  -0.609471;
  g1.grasp_pose.pose.orientation.z =  -0.514462;
  g1.grasp_pose.pose.orientation.w =  0.477894;
  g1.grasp_quality =  32.3708;
  g1.max_contact_force = -1;
  if (!grasp_planning_graspit_ros::writeGraspMessage(g1, output_path, "Grasp_1"))
  {
      PRINTERROR("Could not save to file ");
  }
  
  jpost.positions.resize(3, -0.443135);
  moveit_msgs::Grasp g2;
  g2.id = "jaco_robot_small_cube_1";
  g2.pre_grasp_posture = ttemp;
  g2.pre_grasp_posture.points.push_back(jpre);
  g2.grasp_posture = ttemp;
  g2.grasp_posture.points.push_back(jpost);
  g2.grasp_pose.pose.position.x =  -0.0335148;
  g2.grasp_pose.pose.position.y =  -0.0364662;
  g2.grasp_pose.pose.position.z =  0.188197;
  g2.grasp_pose.pose.orientation.x =  -0.375924;
  g2.grasp_pose.pose.orientation.y =  -0.584158;
  g2.grasp_pose.pose.orientation.z =  -0.5217;
  g2.grasp_pose.pose.orientation.w =  0.495248;
  g2.grasp_quality =  32.4071;
  g2.max_contact_force = -1;
  if (!grasp_planning_graspit_ros::writeGraspMessage(g2, output_path, "Grasp_2"))
  {
      PRINTERROR("Could not save to file ");
  }
  return 0;
}
