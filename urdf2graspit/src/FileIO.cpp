/**
    Copyright (C) 2015 Jennifer Buehler

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
**/
#include <baselib_binding/SharedPtr.h>

#include <urdf2inventor/Helpers.h>
#include <urdf2graspit/FileIO.h>

#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>
  
bool urdf2graspit::FileIO::initOutputDirImpl(const std::string& robotName) const
{
    std::string robotDir;
    return initGraspItRobotDir(robotName,robotDir);
}

bool urdf2graspit::FileIO::initGraspItRobotDir(const std::string& robotName, std::string& robotDir) const
{
    bool ret = urdf_traverser::helpers::makeDirectoryIfNeeded(getOutputDirectory().c_str());

    std::stringstream targetDir;
    targetDir << getOutputDirectory();

    std::vector<std::string> dirs;
    outStructure.getRobotDirPath(dirs);
    for (std::vector<std::string>::const_iterator it = dirs.begin(); it != dirs.end(); ++it)
    {
        targetDir << "/" << *it;
        ret = ret && urdf_traverser::helpers::makeDirectoryIfNeeded(targetDir.str().c_str());
    }
    if (!ret)
    {
        ROS_ERROR("Could not create directory structure in: %s", getOutputDirectory().c_str());
        return false;
    }
    robotDir = targetDir.str();
    return true;
}


std::string urdf2graspit::FileIO::getRobotDir(const std::string& robotName) const
{
    std::stringstream targetDir;
    targetDir << getOutputDirectory() << "/" << outStructure.getRobotDirPath();
    return targetDir.str();
}

bool urdf2graspit::FileIO::writeGraspitMeshFiles(const std::map<std::string, std::string>& meshDescXML) const
{
    std::string outputMeshDir =  getOutputDirectory() + "/" + outStructure.getMeshDirPath();
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(outputMeshDir.c_str()))
    {
        ROS_ERROR("Could not create directory %s", outputMeshDir.c_str());
        return false;
    }

    std::map<std::string, std::string>::const_iterator mdit;
    for (mdit = meshDescXML.begin(); mdit != meshDescXML.end(); ++mdit)
    {
        std::stringstream outXMLFile;
        std::string _outFilename=mdit->first;
        outXMLFile << outputMeshDir << "/" << _outFilename << ".xml";
        if (!urdf_traverser::helpers::writeToFile(mdit->second, outXMLFile.str()))
        {
            ROS_ERROR("Could not write file %s", outXMLFile.str().c_str());
            return false;
        }
    }

    return true;
}

bool urdf2graspit::FileIO::writeEigen(const std::string& robotName, const std::string& content) const
{
    std::string eigenDir = getOutputDirectory() + "/" + outStructure.getEigenGraspDirPath();
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(eigenDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", eigenDir.c_str());
        return false;
    }

    std::string eigenFile = getOutputDirectory() + "/" + outStructure.getEigenGraspFilePath("");
    if (!urdf_traverser::helpers::writeToFile(content, eigenFile))
    {
        ROS_ERROR("Could not write eigengrasp file %s", eigenFile.c_str());
        return false;
    }
    return true;
}


bool urdf2graspit::FileIO::writeContacts(const std::string& robotName, const std::string& content,
    const std::string& filename) const
{
    std::string contactDir = getOutputDirectory() + "/" + outStructure.getContactsDirPath();
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(contactDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", contactDir.c_str());
        return false;
    }
    
    std::string contactFilename = getOutputDirectory() + "/" + outStructure.getContactsFilePath(filename);
    ROS_INFO("Writing contacts to file %s",contactFilename.c_str());
    return urdf_traverser::helpers::writeToFile(content, contactFilename);
}


bool urdf2graspit::FileIO::writeRobotXML(const std::string& robotName, const std::string& content) const
{
    // target dir is robot directory
    std::string outDir = getOutputDirectory() + "/" + outStructure.getRobotDirPath();
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(outDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", outDir.c_str());
        return false;
    }

    std::string robotFile = getOutputDirectory() + "/" + outStructure.getRobotFilePath();
    return urdf_traverser::helpers::writeToFile(content, robotFile);
}


bool urdf2graspit::FileIO::writeWorldFileTemplate(
    const std::string& robotName,
    const std::string& content) const
{
    std::string outDir = getOutputDirectory() + "/" + outStructure.getWorldDirPath();
    if (!urdf_traverser::helpers::makeDirectoryIfNeeded(outDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", outDir.c_str());
        return false;
    }
    std::string outFilename = getOutputDirectory() + "/" + outStructure.getWorldFilePath();
    // ROS_INFO("Writing world file %s",outFilename.c_str());
    return urdf_traverser::helpers::writeToFile(content, outFilename);
}

bool urdf2graspit::FileIO::writeImpl(const ConversionResultPtr& data) const
{
    GraspItConversionResultPtr graspitData = baselib_binding_ns::dynamic_pointer_cast<GraspItConversionResultT>(data);

    if (!graspitData.get())
    {
        ROS_ERROR("Conversion result is not of right type");
        return false;
    }    

    if (!writeRobotXML(graspitData->robotName, graspitData->robotXML))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }

    if (!writeGraspitMeshFiles(graspitData->meshXMLDesc))
    {
        ROS_ERROR("Could not write mesh files");
        return false;
    }

    if (!writeEigen(graspitData->robotName, graspitData->eigenGraspXML))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }

    /*if (!writeContacts(graspitData->robotName, graspitData->contacts))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }*/

    if (!writeWorldFileTemplate(graspitData->robotName, graspitData->world))
    {
        ROS_ERROR("Could not write world file");
        return false;
    }

    return true;
}
