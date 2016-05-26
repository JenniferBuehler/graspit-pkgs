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

#ifndef URDF2GRASPIT_OUTPUTSTRUCTURE_H
#define URDF2GRASPIT_OUTPUTSTRUCTURE_H
// Copyright Jennifer Buehler

#include <string>
#include <vector>

namespace urdf2graspit
{


/**
 * \brief Returns the directory structures and file paths to be used to output the GraspIt! files.
 * \author Jennifer Buehler
 * \date November 2015
 */
class OutputStructure
{
public:
    // \param meshSubdirName the name of the directory where the mesh files will be stored is more flexible
    // than the rest of the structure encoded in this class.
    OutputStructure(const std::string& _meshSubdirName = "iv",
                    const std::string& _texSubdirName = "iv/textures",
                    const std::string& _contactsFile = "contacts.vgr",
                    const std::string& _eigengraspFile = "eigen.xml"):
        robotName("default_robot_name"),
        meshSubdirName(_meshSubdirName),
        texSubdirName(_texSubdirName),
        contactsFile(_contactsFile),
        eigengraspFile(_eigengraspFile) {}

    OutputStructure(const OutputStructure& o):
        robotName(o.robotName),
        meshSubdirName(o.meshSubdirName),
        texSubdirName(o.texSubdirName),
        contactsFile(o.contactsFile),
        eigengraspFile(o.eigengraspFile) {}



    // directory path relative to the root output directory
    void getRobotDirPath(std::vector<std::string>& structure) const;

    // directory path relative to the root output directory
    void getWorldDirPath(std::vector<std::string>& structure) const;

    // directory path relative to the root output directory
    void getEigenGraspDirPath(std::vector<std::string>& structure) const;

    // directory path relative to the root output directory
    void getContactsDirPath(std::vector<std::string>& structure) const;

    // directory path relative to the root output directory
    void getMeshDirPath(std::vector<std::string>& structure) const;
    void getTexDirPath(std::vector<std::string>& structure) const;

    // directory path relative to the root output directory
    std::string getRobotDirPath() const;

    // directory path relative to the root output directory
    std::string getRobotFilePath() const;

    // directory path relative to the root output directory
    std::string getWorldDirPath() const;

    // directory path relative to the root output directory
    std::string getWorldFilePath() const;

    // directory path relative to the root output directory
    std::string getWorldFileName() const;

    // directory path relative to the root output directory
    std::string getEigenGraspDirPath() const;

    /**
     * file path relative to the root output directory.
     * if \e useName is not empty, use this filename instead of the default
     * set in the constructor.
     */
    std::string getEigenGraspFilePath(const std::string& useName) const;

    // directory path relative to the root output directory
    std::string getContactsDirPath() const;

    /**
     * file path relative to the root output directory.
     * if \e useName is not empty, use this filename instead of the default
     * set in the constructor.
     */
    std::string getContactsFilePath(const std::string& useName) const;

    // directory path relative to the root output directory where meshes
    // are to be saved
    std::string getMeshDirPath() const;
    
    // directory path relative to the root output directory where textures
    // are to be saved
    std::string getTexDirPath() const;

    void setRobotName(const std::string& _robotName)
    {
        robotName = _robotName;
    }

    std::string getRobotName() const
    {
        return robotName;
    }

    // relative path to contact file as it should be referenced from the robot's XML file
    std::string getContactsFileRel() const;

    // relative path to eigengrasp file as it should be referenced from the robot's XML file
    std::string getEigenGraspFileRel() const;

    // relative path to mesh directory from the robot's root directory.
    std::string getMeshDirRel() const; 

    // relative path to mesh directory from the robot's root directory.
    std::string getTexDirRel() const; 

private:
    // helper function to convert a path given in a vector to a string
    // in in the form of /path/to/directory.
    std::string toStringPath(const std::vector<std::string>& path) const;

    std::string robotName;
    std::string meshSubdirName;
    std::string texSubdirName;
    std::string contactsFile;
    std::string eigengraspFile;
};


}  // namespace urdf2graspit

#endif  // URDF2GRASPIT_OUTPUTSTRUCTURE_H
