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

#include <urdf2graspit/OutputStructure.h>
#include <urdf_traverser/Helpers.h>

#include <string>
#include <vector>

void urdf2graspit::OutputStructure::getRobotDirPath(std::vector<std::string>& structure) const
{
    structure.push_back("models");
    structure.push_back("robots");
    structure.push_back(robotName);
}


std::string urdf2graspit::OutputStructure::getRobotFilePath() const
{
    return getRobotDirPath() + robotName + ".xml";
}


void urdf2graspit::OutputStructure::getWorldDirPath(std::vector<std::string>& structure) const
{
    structure.push_back("worlds");
}


std::string urdf2graspit::OutputStructure::getWorldFileName() const
{
    return robotName + "_world.xml";
}

std::string urdf2graspit::OutputStructure::getWorldFilePath() const
{
    return getWorldDirPath() + getWorldFileName();
}


void urdf2graspit::OutputStructure::getEigenGraspDirPath(std::vector<std::string>& structure) const
{
    getRobotDirPath(structure);
    structure.push_back("eigen");
}

std::string urdf2graspit::OutputStructure::getEigenGraspFilePath(const std::string& useName) const
{
    return getEigenGraspDirPath() + (useName.empty()? eigengraspFile : useName);
}

std::string urdf2graspit::OutputStructure::getEigenGraspFileRel() const
{
    return "eigen/" + eigengraspFile;
}

void urdf2graspit::OutputStructure::getContactsDirPath(std::vector<std::string>& structure) const
{
    getRobotDirPath(structure);
    structure.push_back("virtual");
}

std::string urdf2graspit::OutputStructure::getContactsFileRel() const
{
    return "virtual/" + contactsFile;
}

std::string urdf2graspit::OutputStructure::getContactsFilePath(const std::string& useName) const
{
    return getContactsDirPath() + (useName.empty()? contactsFile : useName);
}


// ---------------

std::string urdf2graspit::OutputStructure::getMeshDirRel() const
{
    std::string dir = meshSubdirName;
    urdf_traverser::helpers::enforceDirectory(dir, false);
    return dir;
}

std::string urdf2graspit::OutputStructure::getTexDirRel() const
{
    std::string dir = texSubdirName;
    urdf_traverser::helpers::enforceDirectory(dir, false);
    return dir;
}



void urdf2graspit::OutputStructure::getMeshDirPath(std::vector<std::string>& structure) const
{
    getRobotDirPath(structure);
    structure.push_back(meshSubdirName);
}

void urdf2graspit::OutputStructure::getTexDirPath(std::vector<std::string>& structure) const
{
    getRobotDirPath(structure);
    structure.push_back(texSubdirName);
}


std::string urdf2graspit::OutputStructure::getMeshDirPath() const
{
    return getRobotDirPath() + getMeshDirRel();
}

std::string urdf2graspit::OutputStructure::getTexDirPath() const
{
    return getRobotDirPath() + getTexDirRel();
}



std::string urdf2graspit::OutputStructure::toStringPath(const std::vector<std::string>& path) const
{
    std::string ret;
    for (std::vector<std::string>::const_iterator it = path.begin(); it != path.end(); ++it)
    {
        ret += *it +  "/";
    }
    return ret;
}

std::string urdf2graspit::OutputStructure::getRobotDirPath() const
{
    std::vector<std::string> robDir;
    getRobotDirPath(robDir);
    return toStringPath(robDir);
}



std::string urdf2graspit::OutputStructure::getEigenGraspDirPath() const
{
    std::vector<std::string> eDir;
    getEigenGraspDirPath(eDir);
    return toStringPath(eDir);
}


std::string urdf2graspit::OutputStructure::getContactsDirPath() const
{
    std::vector<std::string> cDir;
    getContactsDirPath(cDir);
    return toStringPath(cDir);
}

std::string urdf2graspit::OutputStructure::getWorldDirPath() const
{
    std::vector<std::string> wDir;
    getWorldDirPath(wDir);
    return toStringPath(wDir);
}
