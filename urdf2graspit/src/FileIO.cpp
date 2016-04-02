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

#include <urdf2graspit/Helpers.h>
#include <urdf2graspit/FileIO.h>


#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>


bool urdf2graspit::FileIO::initOutputDir() const
{
    return urdf2graspit::helpers::makeDirectoryIfNeeded(outputDir.c_str());
}


bool urdf2graspit::FileIO::initGraspItRobotDir(const std::string& robotName, std::string& robotDir) const
{
    bool ret = urdf2graspit::helpers::makeDirectoryIfNeeded(outputDir.c_str());

    std::stringstream targetDir;
    targetDir << outputDir;

    std::vector<std::string> dirs;
    outStructure.getRobotDirPath(dirs);
    for (std::vector<std::string>::const_iterator it = dirs.begin(); it != dirs.end(); ++it)
    {
        targetDir << "/" << *it;
        ret = ret && urdf2graspit::helpers::makeDirectoryIfNeeded(targetDir.str().c_str());
    }
    if (!ret)
    {
        ROS_ERROR("Could not create directory structure in: %s", outputDir.c_str());
        return false;
    }
    robotDir = targetDir.str();
    return true;
}


std::string urdf2graspit::FileIO::getRobotDir(const std::string& robotName) const
{
    std::stringstream targetDir;
    targetDir << outputDir << "/" << outStructure.getRobotDirPath();
    return targetDir.str();
}

void findAndReplace(const std::string& newStr, const std::string& oldStr, const std::string& in, std::string& out){
  size_t pos = 0;
  out=in;
  while((pos = out.find(oldStr, pos)) != std::string::npos){
     out = out.replace(pos, oldStr.length(), newStr);
     pos += newStr.length();
  }
}

bool urdf2graspit::FileIO::writeMeshFiles(const std::string& robotName,
        const std::map<std::string, MeshFormat>& meshes,
        const std::map<std::string, std::string>& meshDescXML,
        const std::string& meshOutputExtension,
        const std::string& meshOutputDirectoryName) const
{
    std::string robotDir;
    if (!initGraspItRobotDir(robotName, robotDir))
    {
        ROS_ERROR("Could not create robot directory structure");
        return false;
    }


    std::string outputMeshDir =  outputDir + "/" + outStructure.getMeshDirPath();
    if (!urdf2graspit::helpers::makeDirectoryIfNeeded(outputMeshDir.c_str()))
    {
        ROS_ERROR("Could not create directory %s", outputMeshDir.c_str());
        return false;
    }

    // first, write the mesh files
    std::map<std::string, MeshFormat>::const_iterator mit;
    for (mit = meshes.begin(); mit != meshes.end(); ++mit)
    {
        std::stringstream outFilename;
        std::string _outFilename=mit->first;
        //findAndReplace("_", "/", _outFilename, _outFilename);
        outFilename << outputMeshDir << "/" << _outFilename << meshOutputExtension;

        std::string pathToFile=urdf2graspit::helpers::getPath(outFilename.str().c_str()); 
        //ROS_INFO_STREAM("Directory of path "<<outFilename.str()<<": "<<pathToFile);
        if (!pathToFile.empty() &&
            !urdf2graspit::helpers::makeDirectoryIfNeeded(pathToFile.c_str()))
        {
            ROS_ERROR_STREAM("Could not make directory "<<pathToFile);
        }

        if (!urdf2graspit::helpers::writeToFile(mit->second, outFilename.str()))
        {
            ROS_ERROR("Could not write mesh file %s", outFilename.str().c_str());
            return false;
        }
    }

    std::map<std::string, std::string>::const_iterator mdit;
    for (mdit = meshDescXML.begin(); mdit != meshDescXML.end(); ++mdit)
    {
        std::stringstream outXMLFile;
        std::string _outFilename=mdit->first;
        //findAndReplace("_", "/", _outFilename, _outFilename);
        outXMLFile << outputMeshDir << "/" << _outFilename << ".xml";
        if (!urdf2graspit::helpers::writeToFile(mdit->second, outXMLFile.str()))
        {
            ROS_ERROR("Could not write file %s", outXMLFile.str().c_str());
            return false;
        }
    }

    return true;
}


bool urdf2graspit::FileIO::writeEigen(const std::string& robotName, const std::string& content) const
{
    std::string robotDir;
    if (!initGraspItRobotDir(robotName, robotDir))
    {
        ROS_ERROR("Could not create robot directory structure");
        return false;
    }

    std::string eigenDir = outputDir + "/" + outStructure.getEigenGraspDirPath();
    if (!urdf2graspit::helpers::makeDirectoryIfNeeded(eigenDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", eigenDir.c_str());
        return false;
    }

    std::string eigenFile = outputDir + "/" + outStructure.getEigenGraspFilePath();
    if (!urdf2graspit::helpers::writeToFile(content, eigenFile))
    {
        ROS_ERROR("Could not write eigengrasp file %s", eigenFile.c_str());
        return false;
    }
    return true;
}


bool urdf2graspit::FileIO::writeContacts(const std::string& robotName, const std::string& content) const
{
    std::string robotDir;
    if (!initGraspItRobotDir(robotName, robotDir))
    {
        ROS_ERROR("Could not create robot directory structure");
        return false;
    }

    std::string contactDir = outputDir + "/" + outStructure.getContactsDirPath();
    if (!urdf2graspit::helpers::makeDirectoryIfNeeded(contactDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", contactDir.c_str());
        return false;
    }
    std::string contactFilename = outputDir + "/" + outStructure.getContactsFilePath();
    // ROS_INFO("Writing contacts file %s",contactFilename.c_str());
    return urdf2graspit::helpers::writeToFile(content, contactFilename);
}


bool urdf2graspit::FileIO::writeRobotXML(const std::string& robotName, const std::string& content) const
{
    std::string robotDir;
    if (!initGraspItRobotDir(robotName, robotDir))
    {
        ROS_ERROR("Could not create robot directory structure");
        return false;
    }

    // target dir is robot directory
    std::string outDir = outputDir + "/" + outStructure.getRobotDirPath();
    if (!urdf2graspit::helpers::makeDirectoryIfNeeded(outDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", outDir.c_str());
        return false;
    }

    std::string robotFile = outputDir + "/" + outStructure.getRobotFilePath();
    return urdf2graspit::helpers::writeToFile(content, robotFile);
}


bool urdf2graspit::FileIO::writeWorldFileTemplate(
    const std::string& robotName,
    const std::string& content) const
{
    std::string outDir = outputDir + "/" + outStructure.getWorldDirPath();
    if (!urdf2graspit::helpers::makeDirectoryIfNeeded(outDir.c_str()))
    {
        ROS_ERROR("Could not make directory %s", outDir.c_str());
        return false;
    }
    std::string outFilename = outputDir + "/" + outStructure.getWorldFilePath();
    // ROS_INFO("Writing world file %s",outFilename.c_str());
    return urdf2graspit::helpers::writeToFile(content, outFilename);
}



bool urdf2graspit::FileIO::write(const ConversionResultT& data) const
{
    // First of all, see if we can create output directory
    if (!initOutputDir())
    {
        ROS_ERROR("Can't make directory %s", outputDir.c_str());
        return false;
    }

    if (!writeRobotXML(data.robotName, data.robotXML))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }


    if (!writeMeshFiles(data.robotName, data.meshes, data.meshXMLDesc,
                        data.meshOutputExtension, data.meshOutputDirectoryName))
    {
        ROS_ERROR("Could not write mesh files");
        return false;
    }

    if (!writeEigen(data.robotName, data.eigenGraspXML))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }

    if (!writeContacts(data.robotName, data.contacts))
    {
        ROS_ERROR("Could not write EigenGrasp file");
        return false;
    }

    if (!writeWorldFileTemplate(data.robotName, data.world))
    {
        ROS_ERROR("Could not write world file");
        return false;
    }

    return true;
}


