#ifndef GRASP_PLANNING_GRASPIT_ROS_LOGBINDINGROS_H
#define GRASP_PLANNING_GRASPIT_ROS_LOGBINDINGROS_H
/**
   Logging class implementation for ROS

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

#include <grasp_planning_graspit/LogBinding.h>
#include <ros/ros.h>
#include <string>

namespace GraspIt
{

/**
 * \brief Class to bind to the ROS message printing.
 * \author Jennifer Buehler
 * \date January 2016
 */

class ROSLog: public Log
{
protected:
    virtual void implPrint(const std::stringstream& str)
    {
        ROS_INFO_STREAM(str.str());
    }
    virtual void implPrintError(const std::stringstream& str)
    {
        ROS_ERROR_STREAM(str.str());
    }
    virtual void implPrintWarn(const std::stringstream& str)
    {
        ROS_WARN_STREAM(str.str());
    }

    virtual void implPrint(const char* str)
    {
        ROS_INFO_STREAM(str);
    }
    virtual void implPrintError(const char* str)
    {
        ROS_ERROR_STREAM(str);
    }
    virtual void implPrintWarn(const char* str)
    {
        ROS_WARN_STREAM(str);
    }

    virtual void printNewLine(bool errorStream)
    {
    }
};

}  // namespace GraspIt


#define PRINT_INIT_ROS() \
{\
    if (GraspIt::Log::Singleton) \
    { \
        std::cerr << "Singleton already set, overwriting!" << std::endl;\
    } \
    GraspIt::Log::Singleton = SHARED_PTR<GraspIt::Log>(new GraspIt::ROSLog()); \
}

#endif  // GRASP_PLANNING_GRASPIT_ROS_LOGBINDINGROS_H
