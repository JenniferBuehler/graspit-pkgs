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

#ifndef URDF2GRASPIT_CONVERSIONRESULT_H
#define URDF2GRASPIT_CONVERSIONRESULT_H
// Copyright Jennifer Buehler

#include <string>
#include <map>
#include <urdf2inventor/ConversionResult.h>

namespace urdf2graspit
{

class ConversionParameters: 
    public urdf2inventor::ConversionParameters
{
public:
    explicit ConversionParameters(const std::string& _robotName,
        const std::string& _rootLinkName,
        const std::string& material,
        const std::vector<std::string>& _fingerRoots):
        urdf2inventor::ConversionParameters(_rootLinkName,material),
        robotName(_robotName),
        fingerRoots(_fingerRoots){}
    ConversionParameters(const ConversionParameters& o):
        urdf2inventor::ConversionParameters(o),
        robotName(o.robotName),
        fingerRoots(o.fingerRoots) {}

    virtual ~ConversionParameters() {}
    std::string robotName;
    std::vector<std::string> fingerRoots;
};


/**
 * \brief Encapsulates all result fields for a conversion
 * \author Jennifer Buehler
 * \date November 2015
 */
class ConversionResult:
    public urdf2inventor::ConversionResult<std::string>
{
public:
    typedef std::string MeshFormatT;

    explicit ConversionResult(const std::string& _meshOutputExtension, const std::string& _meshOutputDirectoryName):
        urdf2inventor::ConversionResult<MeshFormatT>(_meshOutputExtension, _meshOutputDirectoryName){}
    ConversionResult(const ConversionResult& o):
        urdf2inventor::ConversionResult<MeshFormatT>(o),
        robotName(o.robotName),
        robotXML(o.robotXML),
        meshXMLDesc(o.meshXMLDesc),
        eigenGraspXML(o.eigenGraspXML),
        contacts(o.contacts),
        world(o.world){}

    virtual ~ConversionResult() {}

    std::string robotName;

    std::string robotXML;

    // the resulting GraspIt! XML description files for the meshes, indexed by the link name
    std::map<std::string, std::string> meshXMLDesc;

    // XML description of the EigenGrasp for the hand
    std::string eigenGraspXML;

    // the content of the file to specify contacts
    std::string contacts;

    // the contents of the XML world file loading up the robot
    std::string world;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_CONVERSIONRESULT_H
