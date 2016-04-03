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

#ifndef URDF2GRASPIT_FILEIO_H
#define URDF2GRASPIT_FILEIO_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>
#include <urdf2graspit/ConversionResult.h>
#include <urdf2graspit/OutputStructure.h>

#include <urdf2inventor/FileIO.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

namespace urdf2graspit
{

/**
 * \brief Reads and writes URDF and GraspIt! files to disk.
 * \author Jennifer Buehler
 * \date October 2015
 */
class FileIO: public urdf2inventor::FileIO<std::string>
{
private:
    typedef urdf2inventor::FileIO<std::string> ParentT;
public:
    
    typedef urdf2graspit::ConversionResult GraspItConversionResultT;
    typedef architecture_binding::shared_ptr<GraspItConversionResultT>::type GraspItConversionResultPtr;

    /**
     * \param _outputDir directory where to save the files. Within this directory, a sub-directory "models/robot/<robotName>" is created,
     * in which all files are saved.
     * Also, within this directory, a file worlds/{robotName}_world.xml is created which can be used as starting point for world files.
     */
    explicit FileIO(const std::string& _outputDir, const OutputStructure& _outStructure):
        urdf2inventor::FileIO<std::string>(_outputDir),
        outStructure(_outStructure) {}

    ~FileIO()
    {
    }

protected:

    virtual bool initOutputDirImpl(const ConversionResultPtr& data) const;

    virtual bool writeImpl(const ConversionResultPtr& data) const;

    bool writeGraspitMeshFiles(const std::map<std::string, std::string>& meshDescXML) const;

    bool writeWorldFileTemplate(const std::string& robotName,
                                const std::string& content) const;

    bool writeEigen(const std::string& robotName, const std::string& content) const;

    bool writeRobotXML(const std::string& robotName, const std::string& content) const;

    bool writeContacts(const std::string& robotName, const std::string& content) const;

    /**
     * Creates the standard GraspIt! directory structure for the robot.
     * in the desired output directory (passed in constructor).
     * \param robotDir the path to the robot directory created (on success) will be returned here.
     * This is the root directory where all the files (robot, constact, eigen, etc.) for this robot will be generated.
     */
    bool initGraspItRobotDir(const std::string& robotName, std::string& robotDir) const;

    std::string getRobotDir(const std::string& robotName) const;

    OutputStructure outStructure;
};
}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_FILEIO_H
