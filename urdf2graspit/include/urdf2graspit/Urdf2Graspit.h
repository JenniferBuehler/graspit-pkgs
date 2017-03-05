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

#ifndef URDF2GRASPIT_URDF2GRASPIT_H
#define URDF2GRASPIT_URDF2GRASPIT_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <baselib_binding/SharedPtr.h>
#include <urdf2graspit/Urdf2GraspItBase.h>

#include <urdf2graspit/DHParam.h>
#include <urdf2graspit/MarkerSelector.h>
#include <urdf2graspit/ConversionResult.h>
#include <urdf2graspit/OutputStructure.h>

// epsilon value to use when comparing whether two axes are the same
// (difference of angle between axes)
#define U2G_EPSILON 1e-07

namespace urdf2graspit
{

/**
 * \brief This class provides functions to transform a robot described in URDF to its description in the GraspIt! format.
 *
 * This requires a few steps:
 * <ol>
 * <li> Conversion of all mesh formats into Inventor format</li>
 * <li> Removal of fixed links in the URDF (fixed links are not fully supported by GraspIt! yet)</li>
 * <li> Transformation of joint transforms such that the rotation axis is always z (to allow for conversion into Denavit-Hartenberg Parameters)</li>
 * <li> Conversion of the joint transforms to the Denavit-Hartenberg convention.</li>
 * <li> (optional) scaling of the model.</li>
 * <li> Generate contact information for the hand model.</li>
 * <li> Generation of all required xml and inventor files.</li>
 * </ol>
 * \author Jennifer Buehler
 */
class Urdf2GraspIt: public Urdf2GraspItBase
{
public:
    typedef urdf2graspit::ConversionResult GraspItConversionResult;
    typedef baselib_binding::shared_ptr<GraspItConversionResult>::type GraspItConversionResultPtr;

    typedef urdf2graspit::ConversionParameters GraspItConversionParameters;
    typedef baselib_binding::shared_ptr<GraspItConversionParameters>::type GraspItConversionParametersPtr;


    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     * \param _negateJointMoves negates (inverts) joint limits and velocity/effort specified in URDF
     */
    explicit Urdf2GraspIt(UrdfTraverserPtr& traverser, float _scaleFactor = 1000, bool _negateJointMoves = false):
        Urdf2GraspItBase(traverser, _scaleFactor),
        negateJointMoves(_negateJointMoves),
        isDHScaled(false),
        dhTransformed(false)
    {
        urdf2inventor::Urdf2Inventor::MESH_OUTPUT_DIRECTORY_NAME = "iv";
    }

    ~Urdf2GraspIt()
    {
    }

    /**
     * Convenience method which does all the operations required for the conversion. This method can be used as a template to create
     * variations of this process. You may be interested in calling cleanup() after processing this in order to clean out temporary
     * files which have been created.
     *
     * a) join the links with fixed joints in the URDF, such that the model does not contain any more fixed joints.
     * b) transform the model such that all rotation axises are the z-axis
     * c) starts a viewer so the user can select contact points in the hand
     * d) calls convert() to transform the model and generate XML files, and writes all necessary files.
     *
     * \param urdfFilename urdf file with the robot description.
     * \param rootLinkName the name of the link in the URDF which will be the root of the GraspIt! model. usually, this is the palm.
     * \param fingerRootNames the roots of all fingers in the hand (the names of the joints which are the first in each finger)
     * \param material the material to use in the converted format
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    ConversionResultPtr processAll(const std::string& urdfFilename,
                                 const std::string& rootLinkName,
                                 const std::vector<std::string>& fingerRootNames,
                                 const std::string& material/* = "plastic"*/,
                                 const EigenTransform& addVisualTransform);


    /**
     * Returns the Denavit-Hartenberg parameters starting from the link \e fromLinkName.
     * DH parameters are returned *for each child joint of fromLinkName*, where each child joint
     * is going to be treated as a root joint starting at the origin.
     *
     * TODO: Provide a function like this, and also toDenavitHartenberg(), which starts at one
     * root joint instead. At the moment the existing solution is tailored for the needs to convert
     * to GraspIt!, but the code can be changed relatively easily to start from a root joint.
     *
     * This will only work if prepareModelForDenavitHartenberg() has been called before.
     */
    bool getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName) const;


    /**
     * Transforms the model to denavit hartenberg parameters, starting from the specified link.
     * See also comments in getDHParams().
     *
     * This method will change some transforms of joints/links and visuals/inertails/collisions.
     * The model must have been transformed before with prepareModelForDenavitHartenberg()!
     */
    bool toDenavitHartenberg(const std::string& fromLink);

private:

    bool checkConversionPrerequisites(const GraspItConversionParametersPtr& param) const;

    virtual ConversionResultPtr preConvert(const ConversionParametersPtr& params);
    virtual ConversionResultPtr postConvert(const ConversionParametersPtr& rootLink, ConversionResultPtr& result);

    /**
     * Returns the Denavit-Hartenberg parameters starting from the joint.
     * This is a recursive function, so it will solve for the requested joint and then call itself on sub-branches
     * \param parentWorldTransform the parent of this joint has this world transform.
     * \param parentX the x-axis as determined by the call of this function for this joint's parent,
     * or the root x axis for the first call
     * \param parentZ the z-axis (rotation axis) as determined by the call of this function for this joint's parent,
     * or the root z axis for the first call
     * \param parentPos position of parent  as determined by the call of this function for this joint's parent,
     * or the origin for the first call
     * \param asRootJoint set to true for the first call of this function
     */
    bool getDHParams(std::vector<DHParam>& dhparameters, const JointConstPtr& joint,
                     const EigenTransform& parentWorldTransform,
                     const Eigen::Vector3d& parentX, const Eigen::Vector3d& parentZ,
                     const Eigen::Vector3d parentPos, bool asRootJoint) const;

    /**
     * Returns the Denavit-Hartenberg parameters starting from link from_link
     */
    bool getDHParams(std::vector<DHParam>& dhparameters, const LinkConstPtr& from_link) const;


    /**
     * scales r and d parameters in \e dh by the given factor
     */
    void scaleParams(std::vector<DHParam>& dh, double scale_factor) const;

    /**
     * Prints the DH parameters
     */
    void printParams(const std::vector<DHParam>& dh) const;

    /**
     * Transforms all link's visuals, collisions and intertials according to the DH parameters.
     * This is needed because the sequence of DH transforms is not equal to the sequence of URDF joint transforms.
     * And the reference frame for a joint in DH parameters is acutally located at the next joint's position (and oriented
     * according to the DH parameter).
     * In URDF, a link is located in the current joint's reference frame as given in the joint's transform.
     * However the link now has to be rooted in the DH reference frame, and oriented such that it
     * ends up at the same global position as it would be when rooted in the URDF joint reference frame.
     * Hence, we have to find for each joint the transform from the joint reference frame to the DH reference frame, and transform
     * the visuals/collisions/intertials by it.
     */
    bool linksToDHReferenceFrames(const std::vector<DHParam>& dh);


    /**
     * returns minimum / maximum limits of this joint. For revolute
     * Joints values in degrees are returned, for prismatic values in mm.
     */
    void getLimits(const urdf::Joint& j, float& min, float& max);

    /**
     * returns velocity and effort of this joint
     */
    void getJointMoves(const urdf::Joint& j, float& velocity, float& effort);


    /**
     * Reads robot information from the DH parameters and from the urdf and returns the GraspIt! XML file as string.
     * Only hands with purely rigid DOF's (only type "r") supported so far. No fixed links are allowed in the URDF,
     * remove them with joinFixedLinks().
     * \param dhparams dh parameters for all joints starting from palmLinkName
     * \param rootFingerJoints a vector of names for the joints which start a finger
     * \param palmLinkName the name of the palm
     * \param mesh_pathprepend path to prepend to the <linkfilename>.xml file path specifications
     */
    bool getXML(const std::vector<DHParam>& dhparams, const std::vector<std::string>& rootFingerJoints,
                const std::string& palmLinkName, const std::string * eigenXML,
                const std::string * contactsVGR, const std::string& mesh_pathprepend, std::string& result);

    // Returns if this is a revoluting joint in the URDF description
    inline bool isRevoluting(const JointPtr& joint) const
    {
        return (joint->type == urdf::Joint::REVOLUTE) || (joint->type == urdf::Joint::CONTINUOUS);
    }

    // Returns if this is a continuous joint in the URDF description
    inline bool isContinuous(const JointPtr& joint) const
    {
        return (joint->type == urdf::Joint::CONTINUOUS);
    }

    // Returns if this is a prismatic joint in the URDF description
    inline bool isPrismatic(const JointPtr& joint) const
    {
        return (joint->type == urdf::Joint::PRISMATIC);
    }

    /**
     * \retval -2 on error
     * \retval -1 if the joint has multiple children
     * \retval 0 if the joint has no child joints (it's an end effector joint),
     * \retval 1 if a child joint is returned in parameter "child"
     */
/*    int getChildJoint(const JointPtr& joint, JointPtr& child);
    LinkPtr getChildLink(const JointPtr& joint);
    JointPtr getParentJoint(const JointPtr& joint);
*/


    /**
     * Transforms a vector (input) within a local coordinate system (given by transform) into world coordinates
     */
    void toGlobalCoordinates(const EigenTransform& transform,
                             const Eigen::Vector3d& input, Eigen::Vector3d& output);

    /**
     * Returns the joint transform in global coordinates, given that the world transform for the parent
     * joint is parentWorldTransform.
     * Both rotation axis and joint position are returned in global coordinates.
     */
    void getGlobalCoordinates(const JointConstPtr& joint,
                              const EigenTransform& parentWorldTransform,
                              Eigen::Vector3d& rotationAxis, Eigen::Vector3d& position) const;


    /**
     * Returns the transform in DH-space for this joint, as present in the dh parameters. Returns false if this joint
     * is not represented in the dh parameters.
     */
    bool getDHTransform(const JointPtr& joint, const std::vector<DHParam>& dh, EigenTransform& result) const;

    /**
     * Gets the XML content which has to go into the world file
     */
    std::string getWorldFileTemplate(const std::string& robotName,
                                     const std::vector<DHParam>& dhparams,
                                     const std::string& prependpath);

    /**
     * Obsolete at this stage -- may be of use still in future.
     * Returns the transform from DH space to standard kinematic chain space (as in URDF) for the joint
     * (or vice versa, if dhToKin is set to false).
     * It is necessary to specify the root of the chain of this joint (the root joint of the chain in
     * which joint exists, specified relative to the global coordinate system). The root joint corresponds
     * to the first joint in the DH-Parameters. The DH-parameters passed as parameter may contain
     * several root joints, one for each chain.
     */
    // bool coordsConvert(const JointPtr& joint, const JointPtr& root_joint, const std::vector<DHParam>& dh,
    //                   bool dhToKin, EigenTransform& result);

    bool negateJointMoves;
    bool isDHScaled;
    bool dhTransformed;

    std::vector<DHParam> dh_parameters;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_URDF2GRASPIT_H
