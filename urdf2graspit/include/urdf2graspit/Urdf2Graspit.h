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

#include <Inventor/nodes/SoTransform.h>

#include <urdf2graspit/DHParam.h>
#include <urdf2graspit/MarkerSelector.h>
#include <urdf2graspit/ConversionResult.h>
#include <urdf2graspit/OutputStructure.h>


// epsilon value to use when comparing whether two axes are the same
// (difference of angle between axes)
#define U2G_EPSILON 1e-07

// this is a temporary filename (extension .iv) which is needed for internal usage, but can be deleted after execution.
#define TMP_FILE_IV "/tmp/urdf2graspit_tmp.iv"

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
class Urdf2GraspIt
{
private:
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

public:
    typedef boost::shared_ptr<const urdf::Joint> Joint_Ptr_Const;
    typedef boost::shared_ptr<const urdf::Link> Link_Ptr_Const;
    typedef boost::shared_ptr<urdf::Joint> Joint_Ptr;
    typedef boost::shared_ptr<urdf::Link> Link_Ptr;

    typedef std::string MeshFormat;

    typedef ConversionResult<MeshFormat> ConversionResultT;


    // output file format to convert the meshes to
    static std::string MESH_OUTPUT_EXTENSION;

    // within the output directory specified in the node, another directory is going to be created
    // to contain the mesh files. The name of this directory can be specified here.
    static std::string MESH_OUTPUT_DIRECTORY_NAME;


    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     * \param _negateJointMoves negates (inverts) joint limits and velocity/effort specified in URDF
     */
    explicit Urdf2GraspIt(float _scaleFactor = 1000, bool _negateJointMoves = true):
        scaleFactor(_scaleFactor),
        negateJointMoves(_negateJointMoves),
        isScaled(false),
        dhTransformed(false),
        outStructure(MESH_OUTPUT_DIRECTORY_NAME) {}


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
    ConversionResultT processAll(const std::string& urdfFilename,
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
    ConversionResultT convert(const std::string& robotName,
                              const std::string& palmLinkName,
                              const std::vector<std::string>& fingerRootJoints,
                              const std::string& material);

    /**
     * Loads the URDF from a file
     */
    bool loadModelFromXMLString(const std::string& xmlString);

    bool getModelFromFile(const std::string& filename, std::string& xml_string) const;

    /**
     * Loads the URDF from parameter server
     */
    bool loadModelFromParameterServer()
    {
        return robot.initParam("robot_description");
    }

    /**
     * Removes all fixed links in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link.
     */
    bool joinFixedLinks(const std::string& from_link);

    /**
     * prints the URDF model to standard out.
     */
    bool printModel(const std::string& fromLink);

    /**
     * writes all elements down from fromLink to a file in inventor format.
     * \param filename has to be an inventor filename
     */
    bool writeAsInventor(const std::string& fromLink, const std::string& filename, bool useScaleFactor = true);

    /**
     * recursive function which returns an inventor node for all links down from (and including) from_link.
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     */
    SoNode * getAsInventor(const std::string& fromLink, bool useScaleFactor = true);


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
     * This will however not include the original mesh files. Use convertMeshes() to generate the GraspIt! meshes
     * using a scale factor.
     */
    bool scaleAll();

    /**
     * Convert all meshes starting from fromLinkName into the GraspIt! inventor format, and store them in the given
     * mesh files container.
     * While converting, the mesh files can be scaled by the given factor.
     * \param material the material to use in the converted format
     * \param meshes the resulting meshes (inventor files), indexed by the link names
     * \param meshDescXML the resulting GraspIt! XML description files for the meshes, indexed by the link names
     */
    bool convertMeshes(const std::string& fromLinkName,
                       double scale_factor, const std::string& material,
                       std::map<std::string, MeshFormat>& meshes,
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

    /**
     * Cleans up all temporary files written to disk.
     */
    void cleanup(bool deleteOutputRedirect = true);


protected:
    /**
     * \brief Encapsulates data carried within a recursion (traverseTree() function). At each recursion, parent, link and level fields
     * are set from within traverseTree.
     */
    class RecursionParams
    {
    public:
        typedef boost::shared_ptr<RecursionParams> Ptr;

        RecursionParams(): level(-1) {}
        RecursionParams(Link_Ptr& _parent, Link_Ptr& _link, unsigned int _level):
            parent(_parent),
            link(_link),
            level(_level) {}
        RecursionParams(const RecursionParams& o):
            parent(o.parent),
            link(o.link),
            level(o.level) {}
        virtual ~RecursionParams() {}

        RecursionParams& operator=(const RecursionParams& o)
        {
            parent = o.parent;
            link = o.link;
            level = o.level;
            return *this;
        }

        // Sets the current variables for the recursion state.
        void setParams(const Link_Ptr& _parent, const Link_Ptr& _link, int _level)
        {
            parent = _parent;
            link = _link;
            level = _level;
        }

        Link_Ptr parent;
        Link_Ptr link;
        unsigned int level;
    };

    typedef RecursionParams::Ptr RecursionParamsPtr;

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
    typedef boost::shared_ptr<Contact> Contact_Ptr;

    /**
     * traverses the tree starting from link, but not including link itself, and calls link_cb on each link.
     * \param link_cb returns -1 or 0 if traversal is to be stopped, otherwise 1. -1 is for stop because error,
     * 0 is stop because an expected condition found
     * \return -1 or 0 if traversal was stopped, otherwise 1. -1 is for stop because error, 0 is stop because an expected
     * condition found in callback function
     */
    int traverseTreeTopDown(const Link_Ptr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                            RecursionParamsPtr& params, unsigned int level = 0);

    /**
     * Similar to traverseTree, but traverses bottom-up and provides ability to re-link tree (by traversing it safely such
     * that changes in structure won't matter)
     */
    bool traverseTreeBottomUp(Link_Ptr& link, boost::function< Link_Ptr(Link_Ptr&)> link_cb);

private:
    /**
     * \brief Includes a factor value to be passed on in recursion.
     */
    class FactorRecursionParams: public Urdf2GraspIt::RecursionParams
    {
    public:
        typedef boost::shared_ptr<FactorRecursionParams> Ptr;
        FactorRecursionParams(): RecursionParams(), factor(1.0) {}
        FactorRecursionParams(Urdf2GraspIt::Link_Ptr& _parent,
                              Urdf2GraspIt::Link_Ptr& _link, int _level, double _factor):
            RecursionParams(_parent, _link, _level),
            factor(_factor) {}
        explicit FactorRecursionParams(double _factor):
            RecursionParams(),
            factor(_factor) {}
        FactorRecursionParams(const FactorRecursionParams& o):
            RecursionParams(o),
            factor(o.factor) {}
        virtual ~FactorRecursionParams() {}

        double factor;
    };

    /**
     * \brief Includes parameters to be passed on in recursion when generating meshes.
     */
    class MeshConvertRecursionParams: public FactorRecursionParams
    {
    public:
        typedef boost::shared_ptr<MeshConvertRecursionParams> Ptr;
        MeshConvertRecursionParams(): FactorRecursionParams() {}

        /**
         * \param material the material to use in the converted mesh
         */
        MeshConvertRecursionParams(Urdf2GraspIt::Link_Ptr& _parent,
                                   Urdf2GraspIt::Link_Ptr& _link,
                                   int _level,
                                   double _scale_factor,
                                   const std::string& _material):
            FactorRecursionParams(_parent, _link, _level, _scale_factor),
            material(_material) {}

        MeshConvertRecursionParams(double _scale_factor, const std::string _material):
            FactorRecursionParams(_scale_factor),
            material(_material) {}
        MeshConvertRecursionParams(const MeshConvertRecursionParams& o):
            FactorRecursionParams(o),
            material(o.material),
            resultMeshes(o.resultMeshes),
            resultXMLDesc(o.resultXMLDesc) {}
        virtual ~MeshConvertRecursionParams() {}

        std::string material;

        // the resulting meshes (inventor files), indexed by the link name
        std::map<std::string, MeshFormat> resultMeshes;

        // the resulting GraspIt! XML description files for the meshes, indexed by the link name
        std::map<std::string, MeshFormat> resultXMLDesc;
    };


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
        std::vector<Urdf2GraspIt::Joint_Ptr> dependencyOrderedJoints;

        // Allow splits, i.e. one link has several child joints. if this is set to false,
        // the recursive operation will fail at splitting points.
        bool allowSplits;

        // Only add joints to the result which are active.
        bool onlyActive;
    };


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
    bool getDHParams(std::vector<DHParam>& dhparameters, const Joint_Ptr& joint,
                     const EigenTransform& parentWorldTransform,
                     const Eigen::Vector3d& parentX, const Eigen::Vector3d& parentZ,
                     const Eigen::Vector3d parentPos, bool asRootJoint);


    /**
     * Returns the Denavit-Hartenberg parameters starting from link from_link
     */
    bool getDHParams(std::vector<DHParam>& dhparameters, const Link_Ptr& from_link);


    /**
     * Returns the Denavit-Hartenberg parameters starting from the link with fromLinkName
     */
    bool getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName);

    /**
     * scales r and d parameters by the given factor
     */
    void scaleParams(std::vector<DHParam>& dh, double scale_factor);



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
     * scales the translation part of the joint transform by the given factor
     */
    bool scaleTranslation(Joint_Ptr& joint, double scale_factor);


    /**
     * scales the translation part of the origins of visuals/collisions/inertial by the given factor
     */
    void scaleTranslation(Link_Ptr& link, double scale_factor);


    /**
     * Applies the transformation on the joint transform
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    bool applyTransform(Joint_Ptr& joint, const EigenTransform& trans, bool preMult);

    /**
     * Applies the transformation on the link's visuals, collisions and intertial.
     * \param scaleTransform set to true if the urdf's transforms are to be scaled (using scaleFactor) before applying the transform
     */
    void applyTransform(Link_Ptr& link, const EigenTransform& trans, bool preMult);

    /**
     * \return true if this joint needs a transformation to align its rotation axis with the given axis.
     * In this case the rotation parameter contains the necessary rotation.
     */
    bool jointTransformForAxis(const urdf::Joint& joint, const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation);

    /**
     * Recursively re-arranges all joint-transforms (starting from joint) and visual/collision/intertial
     * rotations such that all joints rotate around z-axis
     */
    bool allRotationsToAxis(Joint_Ptr& joint, const Eigen::Vector3d& axis);

    /**
     * Function to be called during recursion incurred in convertMeshes()
     */
    int convertMesh(RecursionParamsPtr& p);

    /**
     * Function used for recursion by scaleModel().
     */
    int scaleModel(RecursionParamsPtr& p);

    /**
     * Scales the URDF model by this factor. This means all translation parts of the joint transforms are multiplied by this.
     * The mesh files are not touched, but the visual/collision/intertial translations are scaled as well.
     * Meshes can be scaled using convertMeshes().
     */
    bool scaleModelRecursive(double scale_factor);

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

    /**
     * Prints the structure of the URDF to standard out
     */
    bool printModel();

    /**
     * printing a link, incured by recursive printModel().
     */
    int printLink(RecursionParamsPtr& p);

    /**
     * Function which can be used for recursion which returns -1 if there are any inactive joints in the urdf.
     */
    int checkActiveJoints(RecursionParamsPtr& p);

    // returns true if there are any fixed joints down from from_link
    bool hasFixedJoints(Link_Ptr& from_link);

    /**
     * Helper: If the parent joint of this link is fixed, it will be removed, and this link's visual will be
     * connected to the parent link.
     * If the joint was active, the function returns the same link as in the parameter.
     * Otherwise, it returns the pointer to the parent link which now contains
     * this link's visual/collision.
     */
    Link_Ptr joinFixedLinksOnThis(Link_Ptr& link);

    /**
     * Removes all fixed links down the chain in the model by adding visuals and collision geometry to the first parent link which is
     * attached to a non-fixed link.
     */
    bool joinFixedLinks(Link_Ptr& from_link);

    // Function for recursive getDependencyOrderedJoints
    int addJointLink(RecursionParamsPtr& p);

    /**
     * Returns all joints starting from from_joint (including from_joint) within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<Joint_Ptr>& result, const Joint_Ptr& from_joint,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * Returns all joints down from from_link within the tree. This is obtained by depth-first traversal,
     * so all joints in the result won't depend on any joints further back in the result set.
     */
    bool getDependencyOrderedJoints(std::vector<Joint_Ptr>& result, const Link_Ptr& from_link,
                                    bool allowSplits = true, bool onlyActive = true);

    /**
     * Returns all joints between from_link and to_link
     */
    std::vector<Joint_Ptr> getChain(const Link_Ptr& from_link, const Link_Ptr& to_link) const;


    /**
     * Converts a mesh file (given in filename) to an Inventor structure, to which the root is returned.
     * The model may be scaled at the same time using scale_factor.
     */
    SoNode * convertFile(const std::string& filename, double scale_factor);

    /**
     * Get the mesh from link, scale it up by scale_factor, and pack it into an SoNode which is also respects the
     * scale_factor in its translations
     * \param scaleUrdfTransforms set to true if the transforms coming from this urdf model should be scaled up as well.
     * If this is false, only the meshes are scaled.
     */
    SoNode * getAllVisuals(const Link_Ptr link, double scale_factor, bool scaleUrdfTransforms = false);

    // Returns the transform eTrans as SoTransform node
    SoTransform * getTransform(const EigenTransform& eTrans);

    /**
     * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans)
     * and returns the SoSeparator containing parent with the child.
     */
    SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent, EigenTransform& eTrans);

    /**
     * Adds a SoNode (addAsChild) as a child (to parent), transformed by the given transform (eTrans) and
     * returns the SoSeparator containing parent with the child.
     */
    SoSeparator * addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans);

    /**
     * Writes the contents of SoNode into the file of given name.
     */
    bool writeFile(SoNode * node, const std::string& filename);

    /**
     * writes the contents of SoNode into the inventor (*.iv) format and returns the file
     * content as a string.
     */
    bool writeFileString(SoNode * node, std::string& result);

    // Returns if this is an active joint in the URDF description
    inline bool isActive(const Joint_Ptr& joint) const
    {
        return (joint->type == urdf::Joint::REVOLUTE) ||
               (joint->type == urdf::Joint::CONTINUOUS) ||
               (joint->type == urdf::Joint::PRISMATIC);
    }

    // Returns if this is a revoluting joint in the URDF description
    inline bool isRevoluting(const Joint_Ptr& joint) const
    {
        return (joint->type == urdf::Joint::REVOLUTE) || (joint->type == urdf::Joint::CONTINUOUS);
    }

    // Returns if this is a continuous joint in the URDF description
    inline bool isContinuous(const Joint_Ptr& joint) const
    {
        return (joint->type == urdf::Joint::CONTINUOUS);
    }

    // Returns if this is a prismatic joint in the URDF description
    inline bool isPrismatic(const Joint_Ptr& joint) const
    {
        return (joint->type == urdf::Joint::PRISMATIC);
    }

    // Returns the joint's rotation axis as Eigen Vector
    inline Eigen::Vector3d getRotationAxis(const Joint_Ptr& j)
    {
        return Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
    }

    Joint_Ptr getJoint(const std::string& name);
    Link_Ptr getLink(const std::string& name);

    /**
     * \retval -2 on error
     * \retval -1 if the joint has multiple children
     * \retval 0 if the joint has no child joints (it's an end effector joint),
     * \retval 1 if a child joint is returned in parameter "child"
     */
    int getChildJoint(const Joint_Ptr& joint, Joint_Ptr& child);

    Link_Ptr getChildLink(const Joint_Ptr& joint);

    Joint_Ptr getParentJoint(const Joint_Ptr& joint);

    // Transforms a vector (input) within a local coordinate system (given by transform) into world coordinates
    void toGlobalCoordinates(const EigenTransform& transform,
                             const Eigen::Vector3d& input, Eigen::Vector3d& output);

    /**
     * Returns the joint transform in global coordinates, given that the world transform for the parent
     * joint is parentWorldTransform.
     * Both rotation axis and joint position are returned in global coordinates.
     */
    void getGlobalCoordinates(const Joint_Ptr& joint,
                              const EigenTransform& parentWorldTransform,
                              Eigen::Vector3d& rotationAxis, Eigen::Vector3d& position);



    // Scales up the translation part of the transform t by the given factor
    void scaleTranslation(EigenTransform& t, double scale_factor);


    void setTransform(const EigenTransform& t, urdf::Pose& p);
    void setTransform(const EigenTransform& t, Joint_Ptr& joint);

    // Get joint transform to parent
    EigenTransform getTransform(const urdf::Pose& p);

    // Get joint transform to parent
    inline EigenTransform getTransform(const Joint_Ptr& joint)
    {
        return getTransform(joint->parent_to_joint_origin_transform);
    }

    // Get transform to parent link (transform of link's parent joint)
    inline EigenTransform getTransform(const Link_Ptr& link)
    {
        return getTransform(link->parent_joint);
    }

    Eigen::Matrix4d getTransformMatrix(const Link_Ptr& from_link,  const Link_Ptr& to_link);

    inline EigenTransform getTransform(const Link_Ptr& from_link,  const Link_Ptr& to_link)
    {
        return EigenTransform(getTransformMatrix(from_link, to_link));
    }

    EigenTransform getTransform(const Link_Ptr& from_link,  const Joint_Ptr& to_joint);

    /**
     * Returns the transform in DH-space for this joint, as present in the dh parameters. Returns false if this joint
     * is not represented in the dh parameters.
     */
    bool getDHTransform(const Joint_Ptr& joint, const std::vector<DHParam>& dh, EigenTransform& result);

    /**
     * Recursive function which returns an inventor node for all links down from (and including) from_link.
     * \param useScaleFactor if set to true, the model is scaled up using scale factor set in constructor.
     */
    SoNode * getAsInventor(const Link_Ptr& from_link, bool useScaleFactor);

    /**
     * Writes all elements down from from_link to a file in inventor format.
     * \param filename has to be an inventor filename
     */
    bool writeAsInventor(const Link_Ptr& from_link, const std::string& filename, bool useScaleFactor = true);

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
    // bool coordsConvert(const Joint_Ptr& joint, const Joint_Ptr& root_joint, const std::vector<DHParam>& dh,
    //                   bool dhToKin, EigenTransform& result);

    /**
     * Returns default filename for redirecting stdout output. This file will be in the output
     * folder set in the constructor.
     */
    std::string getStdOutRedirectFile();

    urdf::Model robot;

    // The graspit model might ahve to be scaled compared to the urdf model, this is the scale factor which does that.
    float scaleFactor;
    bool negateJointMoves;
    bool isScaled;
    bool dhTransformed;

    std::vector<DHParam> dh_parameters;
    std::vector<Contact_Ptr> contacts;
    std::map<std::string, std::vector<Contact_Ptr> > linkContacts;

    OutputStructure outStructure;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_URDF2GRASPIT_H
