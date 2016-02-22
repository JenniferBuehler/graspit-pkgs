/**
   Implementation of methods for logging

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
#include <grasp_planning_graspit/SharedPtr.h>
#include <string>

SHARED_PTR<GraspIt::Log> GraspIt::Log::Singleton((GraspIt::Log*) NULL);

bool GraspIt::Log::initSglWarningPrinted = false;


std::string getFilenameFromPath(const std::string& path)
{
    std::string filename = path;
    const size_t last_slash_idx = filename.find_last_of("/");
    if (std::string::npos != last_slash_idx)
    {
        filename.erase(0, last_slash_idx + 1);
    }
    return filename;
}

std::string getFileDirectory(const std::string& pathToFile)
{
    std::string filename = pathToFile;
    const size_t last_slash_idx = filename.find_last_of("/");
    if (std::string::npos != last_slash_idx)
    {
        filename.erase(last_slash_idx + 1, filename.length() - last_slash_idx);
    }
    return filename;
}






