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

#ifndef URDF2GRASPIT_XMLFUNCS_H
#define URDF2GRASPIT_XMLFUNCS_H
// Copyright Jennifer Buehler

#include <string>
#include <vector>
#include <urdf2graspit/Types.h>
#include <urdf2graspit/DHParam.h>
#include <ros/ros.h>


/**
 * Helper functions related to XML. Returns an XML according to the GraspIt! format.
 * \author Jennifer Buehler
 * \date last edited November 2015
 */

// global namespace
namespace urdf2graspit
{
namespace xmlfuncs
{
    
/**
 * \brief A chain of links joined by joints, described in DH parameters.
 * \author Jennifer Buehler
 * \date last edited October 2015
 */
class FingerChain
{

public:
    FingerChain(std::vector<DHParam>& p, std::vector<std::string>& lf, std::vector<std::string>& lt):
        prms(p), linkFileNames(lf), linkTypes(lt) {}
    FingerChain(const FingerChain& o): prms(o.prms), linkFileNames(o.linkFileNames), linkTypes(o.linkTypes) {}
    FingerChain& operator=(const FingerChain& o);

    // DH parameters for joints, in same order as appearing in the chain, either revolute or prismatic
    std::vector<DHParam> prms;
    // filenames of .xml files describing the finger links, in same order as appearing in the chain
    std::vector<std::string> linkFileNames;
    // sames size as linkFileNames, specifies for each link if it is linked to the one
    // before by "Revolute", "Prismatic","Universal", "Ball", or "Fixed".
    std::vector<std::string> linkTypes;

    friend std::ostream& operator<<(std::ostream& o, const FingerChain& p);
};

extern std::string getLinkDescXML(const LinkPtr& link,
                                  const std::string& mesh_output_extension,
                                  const std::string& material = "plastic");

/**
 * \param palmTranslation and palmRotation: the transform from the origin of the palm
 * (which is also the origin of the robot) to the base of this chain,
 * which is where the first joint in the chain is placed.
 */
extern std::string getFingerChain(const FingerChain& c, const Eigen::Vector3d& palmTranslation,
                                  const Eigen::Quaterniond& palmRotation, bool negateJointValues);

extern std::string getDOF(float defaultVel, float maxEffort,
                          float kp = 1e+9, float kd = 1e+7,
                          float draggerScale = 20, const std::string& type = "r");


extern std::string getEigenGraspXML(const std::vector<DHParam>& dhparams, bool negateJointValues);

extern std::string getWorldFileTemplate(const std::string& robotName, const std::vector<DHParam>& dhparams,
                                        const std::string& worldFilePathRelToGraspitRoot, bool negateJointValues);

}  // namespace xmlfuncs
}  // namespace urdf2graspit
#endif  // URDF2GRASPIT_XMLFUNCS_H
