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

#ifndef URDF2GRASPIT_URDF2GRASPITBASE_H
#define URDF2GRASPIT_URDF2GRASPITBASE_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf2inventor/Urdf2Inventor.h>
#include <architecture_binding/SharedPtr.h>
#include <urdf2graspit/DHParam.h>
#include <urdf2graspit/OutputStructure.h>

// epsilon value to use when comparing whether two axes are the same
// (difference of angle between axes)
#define U2G_EPSILON 1e-07

namespace urdf2graspit
{

/**
 * \brief Base class for classes to convert from urdf to the GraspIt! format.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class Urdf2GraspItBase: public urdf2inventor::Urdf2Inventor
{
public:
    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     */
    explicit Urdf2GraspItBase(float _scaleFactor = 1000):
        urdf2inventor::Urdf2Inventor(_scaleFactor),
        outStructure("iv")
    {
    }

    ~Urdf2GraspItBase()
    {
    }

    /**
     * Prepares the model for conversion to denavit hartenberg parameters starting from link \e fromLink.
     * This involves changing all rotation axes to be the z axis by calling
     * allRotationsToAxis(), and and joining all fixed links by calling joinFixedLinks().
     */
    bool prepareModelForDenavitHartenberg(const std::string& fromLink);

    void initOutStructure(const std::string& robotName)
    {
        outStructure.setRobotName(robotName);
    }

    OutputStructure getOutStructure() const
    {
        return outStructure;
    }
    
protected:
    /**
     * Checks whether the loaded URDF model is ready for Denavit-Hartenberg representation.
     * This will return true after prepareModelForDenavitHartenberg has been called.
     * TODO: In the current version, prepareModelForDenavitHartenberg() has to be called
     * on the *same* link name, though it should work if it has been called from a link
     * higher in the hierarchy as well, which is a feature to be added in a future version.
     */
    bool isDHReady(const std::string& fromLink) const;

private:

    OutputStructure outStructure;

    /**
     * set to link which has been used in call of prepareModelForDenavitHartenberg().
     * XXX TODO this should be changed to check dynamically from isDHReady(const std::string&).
     */
    std::string dhReadyFrom;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_URDF2GRASPITBASE_H
