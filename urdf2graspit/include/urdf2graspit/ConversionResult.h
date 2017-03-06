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
#include <urdf2inventor/MeshConvertRecursionParams.h>

namespace urdf2graspit
{

class ConversionParameters:
    public urdf2inventor::ConversionParameters
{
public:
    explicit ConversionParameters(const std::string& _robotName,
        const std::string& _rootLinkName,
        const std::string& material,
        const std::vector<std::string>& _fingerRoots,
        const EigenTransform& addVisualTransform):
        urdf2inventor::ConversionParameters(_rootLinkName, material, addVisualTransform),
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
 * \brief Overwrites MeshConvertRecursionParams to include a special transform per link
 */
class GraspitMeshConvertRecursionParams: public urdf2inventor::MeshConvertRecursionParams<std::string>
{
public:
    typedef GraspitMeshConvertRecursionParams Self;
    typedef urdf2inventor::MeshConvertRecursionParams<std::string> Super;
    typedef typename baselib_binding::shared_ptr<Self>::type Ptr;
    explicit GraspitMeshConvertRecursionParams(double _scale_factor, const std::string _material,
                                        const std::string& _extension,
                                        const urdf_traverser::EigenTransform& _addVisualTransform):
        Super(_scale_factor, _material, _extension, _addVisualTransform) {}
    GraspitMeshConvertRecursionParams(const GraspitMeshConvertRecursionParams& o):
        Super(o){}
    virtual ~GraspitMeshConvertRecursionParams() {}

    virtual urdf_traverser::EigenTransform getVisualTransform() const
    {
      ROS_INFO("CALLING CUSTOM");
      return addVisualTransform;
    }
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

    explicit ConversionResult(const std::string& _meshOutputExtension,
            const std::string& _meshOutputDirectoryName,
            const std::string& _texOutputDirectoryName):
        urdf2inventor::ConversionResult<MeshFormatT>(_meshOutputExtension, _meshOutputDirectoryName, _texOutputDirectoryName){}
    ConversionResult(const ConversionResult& o):
        urdf2inventor::ConversionResult<MeshFormatT>(o),
        robotXML(o.robotXML),
        meshXMLDesc(o.meshXMLDesc),
        eigenGraspXML(o.eigenGraspXML),
        // contacts(o.contacts),
        world(o.world){}

    virtual ~ConversionResult() {}

    std::string robotXML;

    // the resulting GraspIt! XML description files for the meshes, indexed by the link name
    std::map<std::string, std::string> meshXMLDesc;

    // XML description of the EigenGrasp for the hand
    std::string eigenGraspXML;

    // the content of the file to specify contacts
    // std::string contacts;

    // the contents of the XML world file loading up the robot
    std::string world;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_CONVERSIONRESULT_H
