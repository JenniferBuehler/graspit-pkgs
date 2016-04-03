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

#include <urdf2inventor/Urdf2Inventor.h>

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
class Urdf2GraspIt: public urdf2inventor::Urdf2Inventor
{
public:
    typedef urdf2graspit::ConversionResult GraspItConversionResult;
    typedef architecture_binding::shared_ptr<GraspItConversionResult>::type GraspItConversionResultPtr;

    typedef urdf2graspit::ConversionParameters GraspItConversionParameters;
    typedef architecture_binding::shared_ptr<GraspItConversionParameters>::type GraspItConversionParametersPtr;


    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     * \param _negateJointMoves negates (inverts) joint limits and velocity/effort specified in URDF
     */
    explicit Urdf2GraspIt(float _scaleFactor = 1000, bool _negateJointMoves = true):
        urdf2inventor::Urdf2Inventor(_scaleFactor),
        negateJointMoves(_negateJointMoves),
        isDHScaled(false),
        isContactsScaled(false),
        dhTransformed(false),
        outStructure("iv")
    {
        urdf2inventor::Urdf2Inventor::MESH_OUTPUT_DIRECTORY_NAME = "iv";
    }

    ~Urdf2GraspIt()
    {
    }

    OutputStructure getOutStructure() const
    {
        return outStructure;
    }

    /**
     * Transform the URDF such that all rotation axises (in the joint's local reference frame) are this axis
     */
    bool allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis);

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
     */
    ConversionResultPtr processAll(const std::string& urdfFilename,
                                 const std::string& rootLinkName,
                                 const std::vector<std::string>& fingerRootNames,
                                 const std::string& material = "plastic");




    /**
     * Method which does the conversion from URDF to GraspIt!. This converts the model to DH parameters (if not previously done),
     * scales up the model using the scale factor specified in the constructor (if not previously done), converts the mesh files,
     * and produces all XML output files.
     *
     * \param robotName name of the robot, mainly used to create folder names and file names
     * \param palmLinkName the name of the palm link in the URDF file
     * \param fingerRootJoints the roots of all fingers in the hand, i.e. the beginningsn of all GraspIt! chains.
     * \param material the material to use in the converted format
     */
    /*ConversionResultPtr convert(const std::string& robotName,
                              const std::string& palmLinkName,
                              const std::vector<std::string>& fingerRootJoints,
                              const std::string& material);*/

    /**
     * Transforms the model to denavit hartenberg parameters, starting from the specified link.
     * This will change some transforms of joints/links and visuals/inertails/collisions,
     * and will also affect the contacts previously calculated with generateContacts.
     */
    bool toDenavitHartenberg(const std::string& fromLink);


    /**
     * Scales the model using the scale factor specified in constructor. This
     * will also affect the contacts previously calculated with generateContacts, and the DH parameters previously
     * calculated in toDenavitHartenberg.
     * This will however not include the original mesh files. Use Urdf2Inventor::convertMeshes() and convertGraspItMeshes() to generate the 
     * inventor and GraspIt! meshes
     * using a scale factor.
     */
    bool scaleAll();

    /**
     * Convert all meshes starting from fromLinkName into the GraspIt! inventor format, and store them in the given
     * mesh files container.
     * While converting, the mesh files can be scaled by the given factor.
     * \param material the material to use in the converted format
     * \param meshDescXML the resulting GraspIt! XML description files for the meshes, indexed by the link names
     */
    bool convertGraspItMeshes(const std::string& fromLinkName,
                       double scale_factor, const std::string& material,
                       std::map<std::string, std::string>& meshDescXML);


    /**
     * Generates contacts file out of the markers defined in the map. This function is provided in case the markers information
     * is obtained in a user-defined way. You may be interested in using generateContactsWithViewer().
     * After contacts have been generated, they have to be put into DH-space for GraspIt!, which is done in toDenavitHartenberg().
     * They also need scaling, if the model is to be scaled, which is done in scaleAll().
     * \param markers a map with the markers.
     */
    bool generateContacts(const std::vector<std::string>& rootFingerJoints, const std::string& palmLinkName,
                          const float coefficient, const markerselector::MarkerSelector::MarkerMap& markers);

    /**
     * Starts a viewer in which the user may select contact points to be defined for the hand. The function generateContacts()
     * is then called so that contacts are defined for the model. Observe that it is less intuitive to call this function
     * after toDenavitHartenberg() or convert() have been called, as the model is not in its original shape then (the viewer
     * does not use denavit hartenberg convention).
     * After contacts have been generated, they have to be put into DH-space for GraspIt!, which is done in toDenavitHartenberg().
     * They also need scaling, if the model is to be scaled, which is done in scaleAll().
     */
    bool generateContactsWithViewer(const std::vector<std::string>& fingerRoots,
                                    const std::string& palmLinkName, float standard_coefficient);


protected:
    /**
      * \brief Helper class to encapsulate contact information
      */
    class Contact
    {
    public:
        Contact(): numFrictionEdges(-1), fingerNum(-1), linkNum(-1), cof(0) {}
        Contact(const Contact& o):
            numFrictionEdges(o.numFrictionEdges), frictionEdges(o.frictionEdges),
            fingerNum(o.fingerNum), linkNum(o.linkNum),
            loc(o.loc), ori(o.ori), norm(o.norm), cof(o.cof) {}

        friend std::ostream& operator<<(std::ostream& o, const Contact& c)
        {
            o << c.linkNum << ", " << c.fingerNum << ": " << c.loc;
            return o;
        }

        // Number of friction edges
        unsigned int numFrictionEdges;
        // Friction edges (6*numFrictionEdges doubles)
        // The 6 values specify friction cone boundary wrenches.
        // See more documentation in function Urdf2GraspIt::setUpFrictionEllipsoid().
        std::vector<double> frictionEdges;
        // Finger number
        int fingerNum;
        // Link number
        int linkNum;
        // Contact (frame) location
        Eigen::Vector3d loc;
        // Contact (frame) orientation
        Eigen::Quaterniond ori;
        // Contact normal (facing away from link)
        Eigen::Vector3d norm;
        // Contact friciton coefficient
        float cof;
    };
    typedef boost::shared_ptr<Contact> ContactPtr;


private:

    /**
     * \brief Recursion data for getting a list of joints, ordered by dependency (no joint depending on others
     * will come before them in the result vector)
     */
    class OrderedJointsRecursionParams: public Urdf2GraspIt::RecursionParams
    {
    public:
        typedef boost::shared_ptr<OrderedJointsRecursionParams> Ptr;
        OrderedJointsRecursionParams(): RecursionParams() {}
        OrderedJointsRecursionParams(bool _allowSplits, bool _onlyActive):
            allowSplits(_allowSplits),
            onlyActive(_onlyActive) {}
        OrderedJointsRecursionParams(const OrderedJointsRecursionParams& o):
            RecursionParams(o) {}
        virtual ~OrderedJointsRecursionParams() {}

        // Result set
        std::vector<Urdf2GraspIt::JointPtr> dependencyOrderedJoints;

        // Allow splits, i.e. one link has several child joints. if this is set to false,
        // the recursive operation will fail at splitting points.
        bool allowSplits;

        // Only add joints to the result which are active.
        bool onlyActive;
    };


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
    bool getDHParams(std::vector<DHParam>& dhparameters, const JointPtr& joint,
                     const EigenTransform& parentWorldTransform,
                     const Eigen::Vector3d& parentX, const Eigen::Vector3d& parentZ,
                     const Eigen::Vector3d parentPos, bool asRootJoint);


    /**
     * Returns the Denavit-Hartenberg parameters starting from link from_link
     */
    bool getDHParams(std::vector<DHParam>& dhparameters, const LinkPtr& from_link);


    /**
     * Returns the Denavit-Hartenberg parameters starting from the link with fromLinkName
     */
    bool getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName);

    /**
     * scales r and d parameters in \e dh by the given factor
     */
    void scaleParams(std::vector<DHParam>& dh, double scale_factor);
   
    /**
     * scales the contacts by given factor
     */ 
    void scaleContacts(double scale_factor);



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
    bool linksToDHReferenceFrames(std::vector<DHParam>& dh);

    /**
     * Applies the transformation on the joint transform
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    bool applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult);

    /**
     * Applies the transformation on the link's visuals, collisions and intertial.
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    void applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult);

    /**
     * \return true if this joint needs a transformation to align its rotation axis with the given axis.
     * In this case the rotation parameter contains the necessary rotation.
     */
    bool jointTransformForAxis(const urdf::Joint& joint, const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation);

    /**
     * Recursively re-arranges all joint-transforms (starting from joint) and visual/collision/intertial
     * rotations such that all joints rotate around z-axis
     */
    bool allRotationsToAxis(JointPtr& joint, const Eigen::Vector3d& axis);

    /**
     * Function to be called during recursion incurred in convertGraspItMeshes()
     */
    int convertGraspItMesh(RecursionParamsPtr& p);

    /**
     * returns minimum / maximum limits of this joint
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


    // Helper function for generateContacts()
    bool generateContactsForallVisuals(const std::string& linkName,
                                       const int linkNum, const int fingerNum,
                                       const float coefficient,
                                       const std::vector<markerselector::MarkerSelector::Marker>& markers);


    // Function for recursive getDependencyOrderedJoints
    int addJointLink(RecursionParamsPtr& p);

    /**
     * Returns all joints starting from from_joint (including from_joint) within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const JointPtr& from_joint,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * Returns all joints down from from_link within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
                                    bool allowSplits = true, bool onlyActive = true);


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

    // Returns the joint's rotation axis as Eigen Vector
    inline Eigen::Vector3d getRotationAxis(const JointPtr& j)
    {
        return Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
    }

    /**
     * \retval -2 on error
     * \retval -1 if the joint has multiple children
     * \retval 0 if the joint has no child joints (it's an end effector joint),
     * \retval 1 if a child joint is returned in parameter "child"
     */
    int getChildJoint(const JointPtr& joint, JointPtr& child);

    LinkPtr getChildLink(const JointPtr& joint);

    JointPtr getParentJoint(const JointPtr& joint);

    // Transforms a vector (input) within a local coordinate system (given by transform) into world coordinates
    void toGlobalCoordinates(const EigenTransform& transform,
                             const Eigen::Vector3d& input, Eigen::Vector3d& output);

    /**
     * Returns the joint transform in global coordinates, given that the world transform for the parent
     * joint is parentWorldTransform.
     * Both rotation axis and joint position are returned in global coordinates.
     */
    void getGlobalCoordinates(const JointPtr& joint,
                              const EigenTransform& parentWorldTransform,
                              Eigen::Vector3d& rotationAxis, Eigen::Vector3d& position);


    /**
     * Returns the transform in DH-space for this joint, as present in the dh parameters. Returns false if this joint
     * is not represented in the dh parameters.
     */
    bool getDHTransform(const JointPtr& joint, const std::vector<DHParam>& dh, EigenTransform& result);

    /**
     * Writes the file for the contacts. The contacts need to be specified in DH parameter space.
     */
    std::string getContactsFileContent(const std::string& robotName);

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
    bool isContactsScaled;
    bool dhTransformed;

    std::vector<DHParam> dh_parameters;
    std::vector<ContactPtr> contacts;
    std::map<std::string, std::vector<ContactPtr> > linkContacts;

    OutputStructure outStructure;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_URDF2GRASPIT_H
