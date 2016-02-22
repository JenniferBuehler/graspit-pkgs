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

#include <urdf2graspit/Helpers.h>

#include <urdf2graspit/Urdf2Graspit.h>
#include <urdf2graspit/XMLFuncs.h>
#include <urdf2graspit/MarkerSelector.h>
#include <urdf2graspit/ContactFunctions.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ivcon/ivconv.h>

#include <Inventor/SoDB.h>      // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodekits/SoNodeKit.h>

#include <map>
#include <vector>
#include <set>

#define RAD_TO_DEG 180/M_PI

// Filename for temporary output file
#define TEMP_STDOUT "/tmp/redirectedStdOut"

using urdf2graspit::Urdf2GraspIt;
using urdf2graspit::markerselector::MarkerSelector;
using urdf2graspit::xmlfuncs::FingerChain;

std::string Urdf2GraspIt::MESH_OUTPUT_EXTENSION = ".iv";

std::string Urdf2GraspIt::MESH_OUTPUT_DIRECTORY_NAME = "iv";

bool Urdf2GraspIt::getXML(const std::vector<DHParam>& dhparams,
                          const std::vector<std::string>& rootFingerJoints,
                          const std::string& palmLinkName,
                          const std::string * eigenXML,
                          const std::string * contactsVGR,
                          const std::string& mesh_pathprepend, std::string& result)
{
    Link_Ptr from_link;
    this->robot.getLink(palmLinkName, from_link);

    if (hasFixedJoints(from_link))
    {
        ROS_ERROR_STREAM("Only URDF formats without fixed joints supported. "
                         << "You can remove fixed joints with function joinFixedLinks()");
        return false;
    }

    std::stringstream str;

    str << "<?xml version=\"1.0\" ?>" << std::endl;
    str << "<robot type=\"Hand\">" << std::endl;
    str << "<palm>" << palmLinkName << ".xml</palm>" << std::endl;

    float def_kp = 1e+9;
    float def_kd = 1e+7;

    for (std::vector<DHParam>::const_iterator it = dhparams.begin(); it != dhparams.end(); ++it)
    {
        // ROS_INFO_STREAM("PRM Joint: "<<*it);
        float minValue, maxValue;
        getLimits(*(it->joint), minValue, maxValue);
        float draggerScale = fabs(maxValue - minValue);
        float velocity, effort;
        getJointMoves(*(it->joint), velocity, effort);
        str << urdf2graspit::xmlfuncs::getDOF(velocity, effort, def_kp, def_kd, draggerScale, "r") << std::endl;
    }
    // now, pick out the dhparam's for each chain
    for (std::vector<std::string>::const_iterator it = rootFingerJoints.begin(); it != rootFingerJoints.end(); ++it)
    {
        // ROS_INFO("Handling root finger %s",it->c_str());
        Joint_Ptr joint = getJoint(*it);
        if (!joint.get())
        {
            ROS_ERROR("Could not find joint %s", it->c_str());
            return false;
        }
        std::vector<Joint_Ptr> chain;
        bool onlyActive = false;
        if (!getDependencyOrderedJoints(chain, joint, false, onlyActive) || chain.empty())
        {
            ROS_ERROR("Could not get joint chain, joint %s", joint->name.c_str());
            return false;
        }
        // ROS_INFO("---root finger joint %s",joint->name.c_str());
        // keep only those DHParams
        std::vector<DHParam> chain_dhparams;
        std::vector<std::string> linkFileNames;
        std::vector<std::string> linkTypes;
        for (std::vector<Joint_Ptr>::iterator cit = chain.begin(); cit != chain.end(); ++cit)
        {
            // ROS_INFO("Chain joint %s",(*cit)->name.c_str());
            for (std::vector<DHParam>::const_iterator prit = dhparams.begin(); prit != dhparams.end(); ++prit)
            {
                // ROS_INFO("DHJoint %s",prit->joint->name.c_str());
                if ((*cit)->name == prit->joint->name)
                {
                    // ROS_INFO("Keep param for joint %s",prit->joint->name.c_str());

                    chain_dhparams.push_back(*prit);
                    std::stringstream linkfilename;
                    linkfilename << mesh_pathprepend << prit->joint->child_link_name << ".xml";
                    linkFileNames.push_back(linkfilename.str());
                    if (prit->joint->type == urdf::Joint::REVOLUTE) linkTypes.push_back("Revolute");
                    else if (prit->joint->type == urdf::Joint::CONTINUOUS) linkTypes.push_back("Prismatic");
                    else if (prit->joint->type == urdf::Joint::PRISMATIC) linkTypes.push_back("Prismatic");
                    else if (prit->joint->type == urdf::Joint::FIXED) linkTypes.push_back("Fixed");
                    else
                    {
                        ROS_ERROR_STREAM("Link type " << prit->joint->type << " not supported");
                        return false;
                    }
                    break;
                }
            }
        }
        // ROS_INFO("Getting transform for joint %s (starting from %s)",joint->name.c_str(),from_link->name.c_str());
        EigenTransform m = getTransform(from_link, joint);

        // the joint's rotation axis must be z!
        Eigen::Vector3d rotAxis(joint->axis.x, joint->axis.y, joint->axis.z);
        Eigen::Vector3d z(0, 0, 1);
        if (std::fabs(acos(rotAxis.dot(z))) > 1e-03)
        {
            ROS_ERROR("Only z-axis rotation is supported, transform the URDF model before!");
            return false;
        }
        Eigen::Vector3d palmTranslation = m.translation();
        Eigen::Quaterniond palmRotation(m.rotation());

        // ROS_INFO_STREAM("Palm rotation: "<<palmRotation);
        std::string chainStr = getFingerChain(FingerChain(chain_dhparams, linkFileNames, linkTypes),
                                              palmTranslation, palmRotation, negateJointMoves);
        str << chainStr;
    }

    if (eigenXML) str << "<eigenGrasps>" << *eigenXML << "</eigenGrasps>" << std::endl;
    if (contactsVGR) str << "<virtualContacts>" << *contactsVGR << "</virtualContacts>" << std::endl;

    str << "</robot>" << std::endl;

    result = str.str();
    return true;
}


void Urdf2GraspIt::toGlobalCoordinates(const EigenTransform& transform,
                                       const Eigen::Vector3d& input, Eigen::Vector3d& output)
{
    EigenTransform wtInv = transform.inverse();
    Eigen::Vector3d res = wtInv * input;    // transform the rotation axis in world coordinate frame
    //  ROS_INFO_STREAM("inverse transform: "<<wtInv<<" applied to "<<input<<" = "<<res);
    output = res;
}


void Urdf2GraspIt::getGlobalCoordinates(const Joint_Ptr& joint,
                                        const EigenTransform& parentWorldTransform,
                                        Eigen::Vector3d& rotationAxis, Eigen::Vector3d& position)
{
    Eigen::Vector3d rotAxis = getRotationAxis(joint);
    EigenTransform jointTransform = getTransform(joint);

    EigenTransform jointWorldTransform = parentWorldTransform * jointTransform;

    EigenTransform wtInv = jointWorldTransform.inverse();

    // ROS_INFO_STREAM("Joint world transform: "<<jointWorldTransform);

    rotationAxis = wtInv.rotation() * rotAxis;   //  transform the rotation axis in world coordinate frame
    rotationAxis.normalize();

    position = jointWorldTransform.translation();
}

bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparameters, const Joint_Ptr& joint,
                               const EigenTransform& parentWorldTransform,
                               const Eigen::Vector3d& parentX, const Eigen::Vector3d& parentZ,
                               const Eigen::Vector3d parentPos, bool asRootJoint)
{
    ROS_INFO_STREAM("Transforming joint "<<joint->name<<" to DH parameters");

    Link_Ptr childLink;
    this->robot.getLink(joint->child_link_name, childLink);
    if (!childLink.get())
    {
        ROS_ERROR("consistency, no child link");
        return false;
    }


    Eigen::Vector3d z, pos, x;

    EigenTransform jointWorldTransform;

    if (asRootJoint)
    {
        // if this is the root joint, we won't consider the joint's
        // previous transforms, as we'll set this one to be the origin
        jointWorldTransform.setIdentity();

        z = getRotationAxis(joint);   // rotation axis is already in world coorinates
        pos = Eigen::Vector3d(0, 0, 0);

        x = parentX;
    }
    else
    {
        EigenTransform jointTransform = getTransform(joint);
        jointWorldTransform = parentWorldTransform * jointTransform;
        getGlobalCoordinates(joint, parentWorldTransform, z, pos);


        DHParam param;
        if (!DHParam::toDenavitHartenberg(param, parentZ, parentX, parentPos, z, pos, x))
        {
            ROS_ERROR("could not obtain dh params");
            return false;
        }

        param.dof_index = dhparameters.size();
        param.joint = getParentJoint(joint);
        dhparameters.push_back(param);
    }


    if (childLink->child_joints.empty())
    {
        // we've come to the end of the chain, so we have to add the parameter for the last frame.
        // while r should technically be the link length, we'll just leave this frame at the previous joint's
        // frame, as the DH parameters for the last joint don't really matter, they only specify the location
        // of a joint that would follow this one.

        DHParam param;
        param.dof_index = dhparameters.size();
        param.joint = joint;

        param.r = 0;
        param.d = 0;
        param.theta = 0;
        param.alpha = 0;

        dhparameters.push_back(param);
        return true;
    }


    for (std::vector<Joint_Ptr>::const_iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, jointWorldTransform, x, z, pos, false)) return false;
    }

    return true;
}



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparameters, const Link_Ptr& from_link)
{
    // first, adjust the transform such that from_link's rotation axis points in z-direction
    Joint_Ptr parentJoint = from_link->parent_joint;
    EigenTransform root_transform;
    root_transform.setIdentity();
    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d pos(0, 0, 0);
    for (std::vector<Joint_Ptr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, root_transform, x, z, pos, true)) return false;
    }

    return true;
}



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName)
{
    Link_Ptr from_link;
    this->robot.getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    return getDHParams(dhparams, from_link);
}

bool Urdf2GraspIt::getDHTransform(const Joint_Ptr& joint, const std::vector<DHParam>& dh, EigenTransform& result)
{
    for (std::vector<DHParam>::const_iterator it = dh.begin(); it != dh.end(); ++it)
    {
        if (it->joint->name == joint->name)
        {
            result = DHParam::getTransform(*it);
            return true;
        }
    }
    return false;
}

/*
bool Urdf2GraspIt::coordsConvert(const Joint_Ptr& joint, const Joint_Ptr& root_joint,
                                 const std::vector<DHParam>& dh, bool dhToKin, EigenTransform& result)
{
    // starting from the root of a chain, these are the current reference frame transforms in the chain,
    // one in DH space, and one in URDF space
    EigenTransform refFrameDH;
    EigenTransform refFrameURDF;
    refFrameDH.setIdentity();
    refFrameURDF.setIdentity();

    Joint_Ptr iter_joint = root_joint;

    bool finish = false;
    int i = 0;
    do
    {
        EigenTransform dhTrans;
        if (!getDHTransform(iter_joint, dh, dhTrans))
        {
            ROS_ERROR("Joint %s not represented in dh parameters", iter_joint->name.c_str());
            return false;
        }

        EigenTransform jointTransform = EigenTransform::Identity();
        if (i > 0) jointTransform = getTransform(iter_joint);

        // ROS_INFO_STREAM("Dh trans for "<<iter_joint->name<<": "<<dhTrans);
        // ROS_INFO_STREAM("Joint trans for "<<iter_joint->name<<": "<<jointTransform);

        refFrameDH = refFrameDH * dhTrans;
        refFrameURDF = refFrameURDF * jointTransform;

        if (iter_joint->name == joint->name)  // we are finished
        {
            if (dhToKin)
                result = refFrameDH.inverse() * refFrameURDF;
            else
                result = refFrameURDF.inverse() * refFrameDH;
            finish = true;
        }
        else
        {
            int cret = getChildJoint(iter_joint, iter_joint);
            if (cret < 0)
            {
                ROS_ERROR("Child joint could not be retrieved");
                return false;
            }
            if (cret == 0)
            {
                ROS_ERROR("Found the chain end, meaning joints %s and %s are not connected",
                          joint->name.c_str(), root_joint->name.c_str());
                return false;
            }
            ++i;
        }
    }
    while (!finish);

    return true;
}
*/




bool Urdf2GraspIt::linksToDHReferenceFrames(std::vector<DHParam>& dh)
{
    // starting from the root of a chain, these are the current reference frame transforms in the chain,
    // one in DH space, and one in URDF space
    EigenTransform refFrameDH;
    EigenTransform refFrameURDF;
    refFrameDH.setIdentity();
    refFrameURDF.setIdentity();

    Joint_Ptr root_joint;
    int chainCnt = -1;
    bool resetChain = false;
    for (std::vector<DHParam>::iterator it = dh.begin(); it != dh.end(); ++it)
    {
        if (resetChain)
        {
            refFrameDH.setIdentity();
            refFrameURDF.setIdentity();
            chainCnt = -1;
        }
        resetChain = false;

        ++chainCnt;
        Joint_Ptr joint = it->joint;
        Link_Ptr childLink;
        this->robot.getLink(joint->child_link_name, childLink);

        if (!childLink.get())
        {
            ROS_ERROR("Every joint must have a child link");
            return false;
        }


        if (childLink->child_joints.empty())
        {
            // this is the end link, and we've defined the end frame to be
            // at the same location as the last joint,
            // so no rotation should be needed?
            resetChain = true;  // reset chain in next iteration
        }

        /*
        // old, alternative code to below (all code before applyTransform()
        //is the new implementation):
        if (chainCnt == 0) root_joint = joint;
        EigenTransform dhSpaceToJointSpace;
        if (!coordsConvert(joint, root_joint, dh, true, dhSpaceToJointSpace))
        {
            ROS_ERROR("Fail");
            return false;
        }
        */
        EigenTransform dhTrans = DHParam::getTransform(*it);

        EigenTransform jointTransform = EigenTransform::Identity();
        if (chainCnt > 0) jointTransform = getTransform(joint);

        // ROS_INFO_STREAM("Dh trans for "<<joint->name<<": "<<dhTrans);
        // ROS_INFO_STREAM("Joint trans for "<<joint->name<<": "<<jointTransform);

        refFrameDH = refFrameDH * dhTrans;
        refFrameURDF = refFrameURDF * jointTransform;

        EigenTransform dhSpaceToJointSpace = refFrameDH.inverse() * refFrameURDF;

        // ROS_INFO_STREAM("Doing transform for joint "<<joint->name<<": "<<dhSpaceToJointSpace);

        bool preApply = true;
        applyTransform(childLink, dhSpaceToJointSpace, preApply);
    }

    return true;
}


bool Urdf2GraspIt::scaleTranslation(Joint_Ptr& joint, double scale_factor)
{
    EigenTransform vTrans = getTransform(joint);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, joint);
}

void Urdf2GraspIt::scaleTranslation(Link_Ptr& link, double scale_factor)
{
    for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        boost::shared_ptr<urdf::Visual> visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        boost::shared_ptr<urdf::Collision> coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        scaleTranslation(vTrans, scale_factor);
        setTransform(vTrans, coll->origin);
    }
    if (!link->inertial.get())
    {
        // ROS_WARN("Link %s  has no inertial",link->name.c_str());
        return;
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    scaleTranslation(vTrans, scale_factor);
    setTransform(vTrans, link->inertial->origin);

    std::map<std::string, std::vector<Contact_Ptr> >::iterator lCnt = linkContacts.find(link->name);
    if (lCnt != linkContacts.end())
    {
        for (std::vector<Contact_Ptr>::iterator it = lCnt->second.begin(); it != lCnt->second.end(); ++it)
        {
            Contact_Ptr c = *it;
            c->loc *= scale_factor;
        }
    }
}





bool Urdf2GraspIt::applyTransform(Joint_Ptr& joint, const EigenTransform& trans, bool preMult)
{
    EigenTransform vTrans = getTransform(joint);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, joint);
}

void Urdf2GraspIt::applyTransform(Link_Ptr& link, const EigenTransform& trans, bool preMult)
{
    // ROS_INFO("applying transform to link %s",link->name.c_str());

    for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        boost::shared_ptr<urdf::Visual> visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        // ROS_INFO_STREAM("a visual for link"<<link->name<<" with transform "<<vTrans);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        boost::shared_ptr<urdf::Collision> coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, coll->origin);
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, link->inertial->origin);

    std::map<std::string, std::vector<Contact_Ptr> >::iterator lCnt = linkContacts.find(link->name);
    if (lCnt != linkContacts.end())
    {
        for (std::vector<Contact_Ptr>::iterator it = lCnt->second.begin(); it != lCnt->second.end(); ++it)
        {
            Contact_Ptr c = *it;
            EigenTransform t = EigenTransform::Identity();
            t.translate(c->loc);
            t.rotate(c->ori);
            // ROS_INFO_STREAM("////// Applying transform "<<trans<<" to "<<c->loc);
            if (preMult) t = trans * t;
            else         t = t * trans;
            c->loc = t.translation();
            c->ori = Eigen::Quaterniond(t.rotation());
            c->norm = trans.rotation() * c->norm;
        }
    }
}


bool equalAxes(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    return (std::fabs(dot - 1.0)) < U2G_EPSILON;
    // float alpha = acos(z1.dot(z2));
    // return (std::fabs(alpha) < U2G_EPSILON);
}


bool Urdf2GraspIt::jointTransformForAxis(const urdf::Joint& joint,
        const Eigen::Vector3d& axis, Eigen::Quaterniond& rotation)
{
    Eigen::Vector3d rotAxis(joint.axis.x, joint.axis.y, joint.axis.z);
    rotAxis.normalize();
    if (equalAxes(rotAxis, axis)) return false;

    rotation = Eigen::Quaterniond::FromTwoVectors(rotAxis, axis);
    // ROS_WARN_STREAM("z alignment: "<<rotation);
    return true;
}

bool Urdf2GraspIt::allRotationsToAxis(const std::string& fromLinkName, const Eigen::Vector3d& axis)
{
    Link_Ptr from_link;
    this->robot.getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    for (std::vector<Joint_Ptr>::iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}


bool Urdf2GraspIt::allRotationsToAxis(Joint_Ptr& joint, const Eigen::Vector3d& axis)
{
    Link_Ptr childLink;
    this->robot.getLink(joint->child_link_name, childLink);

    if (!childLink.get())
    {
        ROS_ERROR("All joints must have a child link!");
        return false;
    }
    Eigen::Quaterniond alignAxis;
    if (jointTransformForAxis(*joint, axis, alignAxis))
    {
        // ROS_INFO("Transforming z for joint %s",joint->name.c_str());
        applyTransform(joint, EigenTransform(alignAxis), false);
        // the link has to receive the inverse transorm, so it stays at the original position
        Eigen::Quaterniond alignAxisInv = alignAxis.inverse();
        applyTransform(childLink, EigenTransform(alignAxisInv), true);

        // now, we have to fix the child joint's (1st order child joints) transform
        // to correct for this transformation.
        for (std::vector<Joint_Ptr>::iterator pj = childLink->child_joints.begin();
                pj != childLink->child_joints.end(); pj++)
        {
            applyTransform(*pj, EigenTransform(alignAxisInv), true);
        }

        // finally, set the rotation axis to the target
        joint->axis.x = axis.x();
        joint->axis.y = axis.y();
        joint->axis.z = axis.z();
    }
    // recurse
    for (std::vector<Joint_Ptr>::iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!allRotationsToAxis(*pj, axis))
        {
            ROS_ERROR("Aborting recursion.");
            return false;
        }
    }
    return true;
}



int Urdf2GraspIt::scaleModel(RecursionParamsPtr& p)
{
    boost::shared_ptr<FactorRecursionParams> param = boost::dynamic_pointer_cast<FactorRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    Link_Ptr link = param->link;
    if (!link.get())
    {
        ROS_ERROR("Recursion parameter must have initialised link!");
        return -1;
    }
    scaleTranslation(link, param->factor);
    Joint_Ptr pjoint = link->parent_joint;
    if (pjoint.get())
    {
        scaleTranslation(pjoint, param->factor);
    }
    return 1;
}


bool Urdf2GraspIt::scaleModelRecursive(double scale_factor)
{
    Link_Ptr root_link = this->robot.root_link_;
    if (!root_link.get())
    {
        ROS_ERROR("No root link");
        return false;
    }

    // do one call of scaleModel(RecursionParams) for the root link
    Link_Ptr parent;  // leave parent NULL
    RecursionParamsPtr p(new FactorRecursionParams(parent, root_link, 0, scale_factor));
    int cvt = scaleModel(p);

    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    return (cvt == 0) ||
           (this->traverseTreeTopDown(root_link, boost::bind(&Urdf2GraspIt::scaleModel, this, _1), p) >= 0);
}



int Urdf2GraspIt::convertMesh(RecursionParamsPtr& p)
{
    // ROS_INFO("convert mesh for %s",link->name.c_str());

    boost::shared_ptr<MeshConvertRecursionParams> param = boost::dynamic_pointer_cast<MeshConvertRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    Link_Ptr link = param->link;

    SoNode * allVisuals = getAllVisuals(link, param->factor);

    if (!allVisuals)
    {
        ROS_ERROR("Could not get visuals");
        return -1;
    }

    std::string resultFileContent;
    if (!writeFileString(allVisuals, resultFileContent))
    {
        ROS_ERROR("Could not get the mesh file content");
        return -1;
    }

    // ROS_INFO_STREAM("Result file content: "<<resultFileContent);

    if (!param->resultMeshes.insert(std::make_pair(link->name, resultFileContent)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh file for link %s to the map", link->name.c_str());
        return -1;
    }

    std::string linkMeshFile = link->name + MESH_OUTPUT_EXTENSION;
    std::string linkXML = urdf2graspit::xmlfuncs::getLinkDescXML(link, linkMeshFile, param->material);
    // ROS_INFO("XML: %s",linkXML.c_str());

    if (!param->resultXMLDesc.insert(std::make_pair(link->name, linkXML)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh description file for link %s to the map", link->name.c_str());
        return -1;
    }

    return 1;
}



bool Urdf2GraspIt::convertMeshes(const std::string& fromLinkName,
                                 double scale_factor, const std::string& material,
                                 std::map<std::string, MeshFormat>& meshes,
                                 std::map<std::string, std::string>& meshDescXML)
{
    Link_Ptr from_link;
    this->robot.getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    // do one call of convertMeshes
    Link_Ptr parent;  // leave parent NULL
    MeshConvertRecursionParams::Ptr meshParams(new MeshConvertRecursionParams(parent, from_link, 0,
            scale_factor, material));

    RecursionParamsPtr p(meshParams);
    int cvt = convertMesh(p);
    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    int ret = (cvt == 0) ||
              (this->traverseTreeTopDown(from_link, boost::bind(&Urdf2GraspIt::convertMesh, this, _1), p) >= 0);

    meshes = meshParams->resultMeshes;
    meshDescXML = meshParams->resultXMLDesc;

    return ret;
}



void Urdf2GraspIt::scaleParams(std::vector<DHParam>& dh, double scale_factor)
{
    for (std::vector<DHParam>::iterator it = dh.begin(); it != dh.end(); ++it)
    {
        it->d *= scale_factor;
        it->r *= scale_factor;
    }
}




bool Urdf2GraspIt::toDenavitHartenberg(const std::string& fromLink)
{
    ROS_INFO("############### Getting DH params");

    std::vector<DHParam> dhparams;
    if (!getDHParams(dhparams, fromLink))
    {
        ROS_ERROR("Could not get DH parameters");
        return false;
    }
    dh_parameters = dhparams;
    dhTransformed = true;

    /*for (std::vector<DHParam>::iterator d=dhparams.begin(); d!=dhparams.end(); ++d) {
        ROS_INFO_STREAM("DH param: "<<*d);
    }*/


    ROS_INFO("############### Transform links to DH reference frames");

    if (!linksToDHReferenceFrames(dhparams))
    {
        ROS_ERROR("Could not adjust transforms");
        return false;
    }

    return true;
}

bool Urdf2GraspIt::scaleAll()
{
    ROS_INFO("############### Scaling up model");

    double scale_factor = scaleFactor;

    // now, scale up all dh parameters to match the scale factor,
    // and also all link/collision/intertial transforms given in the URDF
    scaleParams(dh_parameters, scale_factor);
    if (!scaleModelRecursive(scale_factor))
    {
        ROS_ERROR("Could not scale up the model");
        return false;
    }

    isScaled = true;
    return true;
}

Urdf2GraspIt::ConversionResultT Urdf2GraspIt::convert(const std::string& robotName,
        const std::string& palmLinkName,
        const std::vector<std::string>& fingerRootJoints,
        const std::string& material)
{
    ConversionResultT res(MESH_OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME);
    res.success = false;
    res.robotName = robotName;

    if (!dhTransformed && !toDenavitHartenberg(palmLinkName))
    {
        ROS_ERROR("Could not transform to DH reference frames");
        return res;
    }


    if (!isScaled && !scaleAll())
    {
        ROS_ERROR("Failed to scale model");
    }

    ROS_INFO("############### Converting meshes");

    if (!convertMeshes(palmLinkName, scaleFactor, material, res.meshes, res.meshXMLDesc))
    {
        ROS_ERROR("Could not convert meshes");
        return res;
    }

    ROS_INFO("############### Getting XML");

    res.eigenGraspXML = urdf2graspit::xmlfuncs::getEigenGraspXML(dh_parameters, negateJointMoves);

    // path to eigen file from robot directory, needed below. Has to match the one created here.
    std::string eigenXML = outStructure.getEigenGraspFileRel();
    std::string contactsVGR = outStructure.getContactsFileRel();

    if (!getXML(dh_parameters, fingerRootJoints, palmLinkName, &eigenXML, &contactsVGR, std::string(), res.robotXML))
    {
        ROS_ERROR("Could not get xml");
        return res;
    }

    // ROS_INFO_STREAM("XML: "<<std::endl<<res.robotXML);

    res.world = getWorldFileTemplate(robotName, dh_parameters, outStructure.getRobotFilePath());

    res.success = true;

    return res;
}

void Urdf2GraspIt::getLimits(const urdf::Joint& j, float& min, float& max)
{
    min = j.limits->lower;
    max = j.limits->upper;
    if (negateJointMoves)
    {
        min = -min;
        max = -max;
    }
}

void Urdf2GraspIt::getJointMoves(const urdf::Joint& j, float& velocity, float& effort)
{
    velocity = j.limits->velocity;
    effort = j.limits->effort;
    if (negateJointMoves)
    {
        velocity = -velocity;
        effort = -effort;
    }
}


bool Urdf2GraspIt::printModel(const std::string& fromLink)
{
    // get root link
    Link_Ptr root_link;
    this->robot.getLink(fromLink, root_link);
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new RecursionParams());
    return this->traverseTreeTopDown(root_link, boost::bind(&Urdf2GraspIt::printLink, this, _1), p) >= 0;
}



bool Urdf2GraspIt::printModel()
{
    // get root link
    Link_Ptr root_link = this->robot.root_link_;
    if (!root_link)
    {
        ROS_ERROR("no root link %s", this->robot.getName().c_str());
        return false;
    }

    ROS_INFO("Root link: %s", root_link->name.c_str());

    // go through entire tree
    RecursionParamsPtr p(new RecursionParams());
    return this->traverseTreeTopDown(root_link, boost::bind(&Urdf2GraspIt::printLink, this, _1), p) >= 0;
}


int Urdf2GraspIt::printLink(RecursionParamsPtr& p)
{
    Link_Ptr parent = p->parent;
    Link_Ptr link = p->link;
    unsigned int level = p->level;

    std::stringstream _indent;
    for (unsigned int i = 0; i < level; ++i) _indent << "   ";
    std::string indent = _indent.str();

    std::string pjoint;
    if (link->parent_joint.get()) pjoint = link->parent_joint->name;
    ROS_INFO("%sInformation about %s: parent joint %s", indent.c_str(), link->name.c_str(), pjoint.c_str());

    // get translation
    double x = link->parent_joint->parent_to_joint_origin_transform.position.x;
    double y = link->parent_joint->parent_to_joint_origin_transform.position.y;
    double z = link->parent_joint->parent_to_joint_origin_transform.position.z;
    ROS_INFO("%s Translation: %f %f %f (%f %f %f)",
             indent.c_str(), x, y, z, x * scaleFactor, y * scaleFactor, z * scaleFactor);

    double qx = link->parent_joint->parent_to_joint_origin_transform.rotation.x;
    double qy = link->parent_joint->parent_to_joint_origin_transform.rotation.y;
    double qz = link->parent_joint->parent_to_joint_origin_transform.rotation.z;
    double qw = link->parent_joint->parent_to_joint_origin_transform.rotation.w;
    ROS_INFO("%s Quaternion: %f %f %f %f", indent.c_str(), qx, qy, qz, qw);

    // get rpy
    double roll, pitch, yaw;
    link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

    if (isnan(roll) || isnan(pitch) || isnan(yaw))
    {
        ROS_ERROR("getRPY() returned nan!");
        return -1;
    }

    ROS_INFO("%s  (=RPY: %f %f %f)", indent.c_str(), roll, pitch, yaw);
    return 1;
}


int Urdf2GraspIt::traverseTreeTopDown(const Link_Ptr& link, boost::function< int(RecursionParamsPtr&)> link_cb,
                                      RecursionParamsPtr& params, unsigned int level)
{
    level += 1;
    for (std::vector<Link_Ptr>::const_iterator child = link->child_links.begin();
            child != link->child_links.end(); child++)
    {
        Link_Ptr childLink = *child;
        if (childLink.get())
        {
            params->setParams(link, childLink, level);
            int link_ret = link_cb(params);
            if (link_ret <= 0)
            {
                // stopping traversal
                return link_ret;
            }

            // recurse down the tree
            int ret = traverseTreeTopDown(childLink, link_cb, params, level);
            if (ret < 0)
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return -1;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }
    return 1;
};

bool Urdf2GraspIt::traverseTreeBottomUp(Link_Ptr& link, boost::function< Link_Ptr(Link_Ptr&)> link_cb)
{
    std::set<std::string> toTraverse;
    for (unsigned int i = 0; i < link->child_links.size(); ++i)
    {
        Link_Ptr childLink = link->child_links[i];
        toTraverse.insert(childLink->name);
    }

    for (std::set<std::string>::iterator it = toTraverse.begin(); it != toTraverse.end(); ++it)
    {
        Link_Ptr childLink;
        this->robot.getLink(*it, childLink);

        if (childLink.get())
        {
            // ROS_INFO("Traversal into child %s",childLink->name.c_str());
            // recurse down the tree
            if (!traverseTreeBottomUp(childLink, link_cb))
            {
                ROS_ERROR("Error parsing branch of %s", childLink->name.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }

    // ROS_INFO("Relink child %s",link->name.c_str());
    link = link_cb(link);
    if (!link.get())
    {
        ROS_ERROR("Error parsing branch of %s", link->name.c_str());
        return false;
    }
    return true;
}


int Urdf2GraspIt::checkActiveJoints(RecursionParamsPtr& p)
{
    Link_Ptr parent = p->parent;
    Link_Ptr link = p->link;
    unsigned int level = p->level;
    if (link->parent_joint.get() && !isActive(link->parent_joint))
    {
        ROS_ERROR("C: Found fixed link %s", link->parent_joint->name.c_str());
        return -1;
    }
    return 1;
}

bool Urdf2GraspIt::hasFixedJoints(Link_Ptr& from_link)
{
    Link_Ptr empty;
    Link_Ptr l = from_link;
    RecursionParamsPtr p(new RecursionParams(empty, l, 0));
    if ((checkActiveJoints(p) < 0) ||
            (this->traverseTreeTopDown(from_link,
                                       boost::bind(&Urdf2GraspIt::checkActiveJoints, this, _1), p) < 0))
    {
        return true;
    }
    return false;
}


bool Urdf2GraspIt::joinFixedLinks(const std::string& from_link)
{
    Link_Ptr link;
    this->robot.getLink(from_link, link);
    return joinFixedLinks(link);
}

bool Urdf2GraspIt::joinFixedLinks(Link_Ptr& from_link)
{
    if (!this->traverseTreeBottomUp(from_link, boost::bind(&Urdf2GraspIt::joinFixedLinksOnThis, this, _1)))
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }

    // consistency check: All joints in the tree must be active now!
    if (hasFixedJoints(from_link))
    {
        ROS_ERROR("consistency: We should now only have active joitns in the tree!");
        return false;
    }
    return true;
}


Urdf2GraspIt::Link_Ptr Urdf2GraspIt::joinFixedLinksOnThis(Link_Ptr& link)
{
    if (!link.get()) return link;

    // ROS_INFO("Traverse %s",link->name.c_str());


    Joint_Ptr jointToParent = link->parent_joint;
    if (!jointToParent.get())
    {
        ROS_WARN("End of chain at %s, because of no parent joint", link->name.c_str());
        return link;
    }


    Link_Ptr parentLink;
    this->robot.getLink(jointToParent->parent_link_name, parentLink);
    if (!parentLink.get())
    {
        ROS_WARN("End of chain at %s, because of no parent link", link->name.c_str());
        return link;
    }

    /*if (link->child_joints.empty()) {
        ROS_WARN("INFO: end effector %s",link->name.c_str());
        return link;
    }*/


    if (isActive(jointToParent))
    {
        // ROS_INFO("Parent of %s is active so won't delete",link->name.c_str());
        // We won't delete this joint, as it is active.
        // ROS_INFO("Joining chain finished between %s and %s",parentLink->name.c_str(),link->name.c_str());
        return link;
    }

    // this joint is fixed, so we will delete it

    // ROS_INFO("Joining between %s and %s",parentLink->name.c_str(),link->name.c_str());
    // remove this link from the parent
    for (std::vector<Link_Ptr >::iterator pc = parentLink->child_links.begin();
            pc != parentLink->child_links.end(); pc++)
    {
        Link_Ptr child = (*pc);
        if (child->name == link->name)
        {
            // ROS_WARN("Remove link %s",link->name.c_str());
            parentLink->child_links.erase(pc);
            break;
        }
    }
    for (std::vector<Joint_Ptr >::iterator pj = parentLink->child_joints.begin();
            pj != parentLink->child_joints.end(); pj++)
    {
        Joint_Ptr child = (*pj);
        // this child joint is the current one that we just now removed
        if (child->name == jointToParent->name)
        {
            // ROS_WARN("Remove joint %s",child->name.c_str());
            parentLink->child_joints.erase(pj);
            break;
        }
    }

    // the local transfrom of the parent joint
    EigenTransform localTrans = getTransform(link);

    // all this link's child joints now must receive the extra transform from this joint which we removed.
    // then, the joints should be added to the parent link
    for (std::vector<Joint_Ptr >::iterator j = link->child_joints.begin(); j != link->child_joints.end(); j++)
    {
        Joint_Ptr child = (*j);
        if (!isActive(child))
        {
            ROS_ERROR("consistency: At this stage, we should only have active joints, found joint %s!",
                      child->name.c_str());
        }
        EigenTransform vTrans = getTransform(child);

        vTrans = localTrans * vTrans;
        setTransform(vTrans, child);
        child->parent_link_name = parentLink->name;
        parentLink->child_joints.push_back(child);

        // this link's child link has to be added to parents as well
        Link_Ptr childChildLink;
        this->robot.getLink(child->child_link_name, childChildLink);
        if (!childChildLink.get())
        {
            ROS_ERROR("consistency: found null child link for joint %s", child->name.c_str());
        }
        parentLink->child_links.push_back(childChildLink);
        childChildLink->setParent(parentLink);
    }

    for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        boost::shared_ptr<urdf::Visual> visual = *vit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform vTrans = getTransform(visual->origin);
        vTrans = localTrans * vTrans;
        setTransform(vTrans, visual->origin);
        parentLink->visual_array.push_back(visual);
    }


    for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        boost::shared_ptr<urdf::Collision> coll = *cit;
        // apply the transform to the visual before adding it to the parent
        EigenTransform vTrans = getTransform(coll->origin);
        vTrans = localTrans * vTrans;
        setTransform(vTrans, coll->origin);
        parentLink->collision_array.push_back(coll);
    }


    if (parentLink->visual.get()) parentLink->visual.reset();
    if (parentLink->collision.get()) parentLink->collision.reset();


    // combine inertials
    // parent->inertial=XXX TODO;

    return parentLink;
}


int Urdf2GraspIt::addJointLink(RecursionParamsPtr& p)
{
    boost::shared_ptr<OrderedJointsRecursionParams> param =
        boost::dynamic_pointer_cast<OrderedJointsRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    // ROS_INFO("At link %s", parent->name.c_str());
    if (param->parent->child_joints.empty())
    {
        ROS_ERROR("If links are connected, there must be at least one joint");
        return -1;
    }
    if (!param->link->parent_joint.get())
    {
        ROS_ERROR("NULL parent joint");
        return -1;
    }
    if (param->parent->child_joints.size() > 1)
    {
        if (!param->allowSplits)
        {
            ROS_ERROR("Splitting point at %s!", param->parent->name.c_str());
            return -1;
        }
        // this is a splitting point, we have to add support for this
    }

    if (param->onlyActive && !isActive(param->link->parent_joint))
    {
        // ROS_INFO("No type");
        return 1;
    }


    // ROS_INFO("Adding %s",link->parent_joint->name.c_str());
    param->dependencyOrderedJoints.push_back(param->link->parent_joint);
    return 1;
}


bool Urdf2GraspIt::getDependencyOrderedJoints(std::vector<Joint_Ptr>& result,
        const Joint_Ptr& from_joint, bool allowSplits, bool onlyActive)
{
    Link_Ptr childLink;
    this->robot.getLink(from_joint->child_link_name, childLink);
    if (!childLink.get())
    {
        ROS_ERROR("Child link %s not found", from_joint->child_link_name.c_str());
        return false;
    }
    if (!getDependencyOrderedJoints(result, childLink, allowSplits, onlyActive))
    {
        ROS_ERROR("Could not get ordered joints for %s", from_joint->child_link_name.c_str());
        return false;
    }
    if (!onlyActive || isActive(from_joint))
    {
        result.insert(result.begin(), from_joint);
    }
    return true;
}

bool Urdf2GraspIt::getDependencyOrderedJoints(std::vector<Joint_Ptr>& result, const Link_Ptr& from_link,
        bool allowSplits, bool onlyActive)
{
    if (!allowSplits && (from_link->child_joints.size() > 1))
    {
        ROS_ERROR("Splitting point at %s!", from_link->name.c_str());
        return false;
    }
    OrderedJointsRecursionParams * p = new OrderedJointsRecursionParams(allowSplits, onlyActive);
    RecursionParamsPtr rp(p);
    if (this->traverseTreeTopDown(from_link, boost::bind(&Urdf2GraspIt::addJointLink, this, _1), rp) < 0)
    {
        ROS_ERROR("Could not add depenency order");
        p->dependencyOrderedJoints.clear();
        return false;
    }

    result = p->dependencyOrderedJoints;
    return true;
}


std::vector<Urdf2GraspIt::Joint_Ptr> Urdf2GraspIt::getChain(const Link_Ptr& from_link, const Link_Ptr& to_link) const
{
    std::vector<Joint_Ptr> chain;

    if (to_link->name == from_link->name) return chain;

    Link_Ptr curr = to_link;
    Link_Ptr pl = to_link->getParent();

    while (curr.get() && (curr->name != from_link->name))
    {
        Joint_Ptr pj = curr->parent_joint;
        if (!pj.get())
        {
            ROS_ERROR("Null parent joint found! %s", curr->name.c_str());
            return chain;
        }
        chain.push_back(pj);
        curr = pl;
        pl = curr->getParent();
        // ROS_INFO("Parent of %s %s",curr->name.c_str(),pl->name.c_str());
    }
    if (curr->name != from_link->name)
    {
        ROS_ERROR("Failed to find parent chain!");
        return std::vector<Joint_Ptr>();
    }

    std::reverse(chain.begin(), chain.end());

    return chain;
}


std::string Urdf2GraspIt::getStdOutRedirectFile()
{
    /*std::stringstream str;
    str << outputDir.c_str() << "/" << TEMP_STDOUT;
    return str.str();*/
    return TEMP_STDOUT;
}


SoNode * Urdf2GraspIt::convertFile(const std::string& filename, double scale_factor)
{
    // int ssize=10000;
    // char bigOutBuf[ssize];
    urdf2graspit::helpers::redirectStdOut(getStdOutRedirectFile().c_str());

    // first, convert file to inventor. ivconv writes to file only so we have to
    // temporarily write it to file and then read it again
    IVCONV::SCALE_FACTOR = scale_factor;
    IVCONV ivconv;
    if (!ivconv.read(filename))
    {
        ROS_ERROR("Can't read mesh file %s", filename.c_str());
        return NULL;
    }


    if (!ivconv.write(TMP_FILE_IV))
    {
        ROS_ERROR("Can't write mesh file %s", TMP_FILE_IV);
        return NULL;
    }


    SoInput in;
    SoNode  *scene = NULL;
    if (!in.openFile(TMP_FILE_IV)) return NULL;
    SoDB::read(&in, scene);

    in.closeFile();

    urdf2graspit::helpers::resetStdOut();


    // ROS_INFO("We got %s",bigOutBuf);

    return scene;
}

void Urdf2GraspIt::cleanup(bool deleteOutputRedirect)
{
    const char * stdOutRedirectFile = getStdOutRedirectFile().c_str();
    if (deleteOutputRedirect &&
            urdf2graspit::helpers::fileExists(stdOutRedirectFile))
    {
        urdf2graspit::helpers::deleteFile(stdOutRedirectFile);
    }
    if (urdf2graspit::helpers::fileExists(TMP_FILE_IV))
    {
        urdf2graspit::helpers::deleteFile(TMP_FILE_IV);
    }
}

bool Urdf2GraspIt::writeFile(SoNode * node, const std::string& filename)
{
    SoOutput out;
    if (!out.openFile(filename.c_str())) return false;
    out.setBinary(false);
    SoWriteAction write(&out);
    write.apply(node);
    write.getOutput()->closeFile();
    return true;
}

bool Urdf2GraspIt::writeFileString(SoNode * node, std::string& result)
{
    SoOutput out;
    out.setBinary(false);
    size_t initBufSize = 100;
    void * buffer = malloc(initBufSize * sizeof(char));
    out.setBuffer(buffer, initBufSize, std::realloc);
    SoWriteAction write(&out);
    write.apply(node);

    void * resBuf = NULL;
    size_t resBufSize = 0;

    if (!out.getBuffer(resBuf, resBufSize) || (resBufSize == 0))
    {
        ROS_ERROR("Failed to write file string to buffer.");
        return false;
    }

    result = std::string(static_cast<char*>(resBuf), resBufSize);  // buffer will be copied

    free(resBuf);

    return true;
}





SoSeparator * Urdf2GraspIt::addSubNode(SoNode * addAsChild, SoNode* parent, SoTransform * trans)
{
    SoSeparator * sep = dynamic_cast<SoSeparator*>(parent);
    if (!sep)
    {
        ROS_ERROR("parent is not a separator");
        return NULL;
    }

    SoSeparator * sepChild = dynamic_cast<SoSeparator*>(addAsChild);
    if (!sepChild)
    {
        ROS_ERROR("child is not a separator");
        return NULL;
    }

    // ROS_WARN_STREAM("######### Adding transform "<<trans->translation<<", "<<trans->rotation);

    SoSeparator * transNode = new SoSeparator();
    transNode->addChild(trans);
    transNode->addChild(sepChild);

    sep->addChild(transNode);
    return sep;
}

SoSeparator * Urdf2GraspIt::addSubNode(SoNode * addAsChild, SoNode* parent, EigenTransform& eTrans)
{
    SoTransform * transform = getTransform(eTrans);
    return addSubNode(addAsChild, parent, transform);
}




SoNode * Urdf2GraspIt::getAllVisuals(const Link_Ptr link, double scale_factor, bool scaleUrdfTransforms)
{
    SoNodeKit::init();
    SoNode * allVisuals = new SoSeparator();
    allVisuals->ref();
    std::string linkName = link->name;
    unsigned int i = 0;
    for (std::vector<boost::shared_ptr<urdf::Visual> >::const_iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        boost::shared_ptr<urdf::Visual> visual = (*vit);
        boost::shared_ptr<urdf::Geometry> geom = visual->geometry;

        EigenTransform vTransform = getTransform(visual->origin);
        if (scaleUrdfTransforms) scaleTranslation(vTransform, scale_factor);

        if (geom->type == urdf::Geometry::MESH)
        {
            boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(geom);
            if (!mesh.get())
            {
                ROS_ERROR("Mesh cast error");
                return NULL;
            }
            std::string meshFilename = urdf2graspit::helpers::packagePathToAbsolute(mesh->filename);

            SoNode * somesh = convertFile(meshFilename, scale_factor);
            std::stringstream str;
            str << "_visual_" << i << "_" << linkName;
            somesh->setName(str.str().c_str());
            allVisuals = addSubNode(somesh, allVisuals, vTransform);
        }
        else
        {
            ROS_ERROR("Only support mesh files so far");
            return NULL;
        }
        ++i;
    }

    std::stringstream str;
    str << "_" << linkName;

    allVisuals->setName(str.str().c_str());

    return allVisuals;
}

SoTransform * Urdf2GraspIt::getTransform(const EigenTransform& eTrans)
{
    SoTransform * transform = new SoTransform();

    SoSFVec3f translation;
    translation.setValue(eTrans.translation().x(), eTrans.translation().y(), eTrans.translation().z());
    transform->translation = translation;

    SoSFRotation rotation;
    Eigen::Quaterniond vQuat(eTrans.rotation());
    rotation.setValue(vQuat.x(), vQuat.y(), vQuat.z(), vQuat.w());
    transform->rotation = rotation;
    return transform;
}



Urdf2GraspIt::Link_Ptr Urdf2GraspIt::getLink(const std::string& name)
{
    Link_Ptr ptr;
    this->robot.getLink(name, ptr);
    return ptr;
}


Urdf2GraspIt::Joint_Ptr Urdf2GraspIt::getJoint(const std::string& name)
{
    Joint_Ptr ptr;
    if (this->robot.joints_.find(name) == this->robot.joints_.end()) ptr.reset();
    else ptr = this->robot.joints_.find(name)->second;
    return ptr;
}


int Urdf2GraspIt::getChildJoint(const Joint_Ptr& joint, Joint_Ptr& child)
{
    Link_Ptr childLink = getChildLink(joint);
    if (!childLink.get())
    {
        ROS_ERROR("Consistency: all joints must have child links");
        return -2;
    }
    if (childLink->child_joints.size() > 1)
    {
        return -1;
    }
    if (childLink->child_joints.empty())
    {
        // this is the end link, and we've defined the end frame to be at the same location as the last joint,
        // so no rotation should be needed?
        return 0;
    }
    // there must be only one joint
    child = childLink->child_joints.front();
    return 1;
}

Urdf2GraspIt::Link_Ptr Urdf2GraspIt::getChildLink(const Joint_Ptr& joint)
{
    Link_Ptr childLink;
    this->robot.getLink(joint->child_link_name, childLink);
    return childLink;
}

Urdf2GraspIt::Joint_Ptr Urdf2GraspIt::getParentJoint(const Joint_Ptr& joint)
{
    Link_Ptr_Const parentLink = this->robot.getLink(joint->parent_link_name);
    if (!parentLink.get()) return Joint_Ptr();
    return parentLink->parent_joint;
}


void Urdf2GraspIt::setTransform(const EigenTransform& t, urdf::Pose& p)
{
    Eigen::Vector3d trans(t.translation());
    Eigen::Quaterniond rot(t.rotation());

    p.position.x = trans.x();
    p.position.y = trans.y();
    p.position.z = trans.z();
    p.rotation.x = rot.x();
    p.rotation.y = rot.y();
    p.rotation.z = rot.z();
    p.rotation.w = rot.w();
}

void Urdf2GraspIt::setTransform(const EigenTransform& t, Joint_Ptr& joint)
{
    setTransform(t, joint->parent_to_joint_origin_transform);
}

void Urdf2GraspIt::scaleTranslation(EigenTransform& t, double scale_factor)
{
    Eigen::Vector3d trans = t.translation();
    trans *= scale_factor;
    Eigen::Matrix3d rot = t.rotation();
    t.setIdentity();
    t.translate(trans);
    t.rotate(rot);
}


Urdf2GraspIt::EigenTransform Urdf2GraspIt::getTransform(const urdf::Pose& p)
{
    urdf::Vector3 _jtr = p.position;
    Eigen::Vector3d jtr(_jtr.x, _jtr.y, _jtr.z);
    urdf::Rotation _jrot = p.rotation;
    Eigen::Quaterniond jrot(_jrot.w, _jrot.x, _jrot.y, _jrot.z);
    jrot.normalize();
    EigenTransform tr;
    tr.setIdentity();
    tr = tr.translate(jtr);
    tr = tr.rotate(jrot);
    return tr;
}


Eigen::Matrix4d Urdf2GraspIt::getTransformMatrix(const Link_Ptr& from_link,  const Link_Ptr& to_link)
{
    if (from_link->name == to_link->name) return Eigen::Matrix4d::Identity();

    std::vector<Joint_Ptr> pjoints = getChain(from_link, to_link);

    if (pjoints.empty())
    {
        ROS_ERROR("could not get chain from %s to %s", from_link->name.c_str(), to_link->name.c_str());
        return Eigen::Matrix4d::Identity();
    }

    // ROS_INFO("Chain from %s to %s",from_link->name.c_str(),to_link->name.c_str());

    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();

    for (std::vector<Joint_Ptr>::iterator it = pjoints.begin(); it != pjoints.end(); ++it)
    {
        // ROS_INFO("Chain joint %s",(*it)->name.c_str());
        Eigen::Matrix4d mat = getTransform(*it).matrix();
        ret *= mat;
    }
    return ret;
}

Urdf2GraspIt::EigenTransform Urdf2GraspIt::getTransform(const Link_Ptr& from_link,  const Joint_Ptr& to_joint)
{
    Link_Ptr link1 = from_link;
    Link_Ptr link2;
    this->robot.getLink(to_joint->child_link_name, link2);
    if (!link1.get() || !link2.get())
    {
        ROS_ERROR("Invalid joint specifications (%s, %s), first needs parent and second child",
                  link1->name.c_str(), link2->name.c_str());
    }
    return getTransform(link1, link2);
}


SoNode * Urdf2GraspIt::getAsInventor(const Link_Ptr& from_link, bool useScaleFactor)
{
    SoNode * allVisuals = getAllVisuals(from_link, useScaleFactor ? scaleFactor : 1.0, useScaleFactor);

    for (std::vector<Joint_Ptr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        Link_Ptr childLink;
        this->robot.getLink((*pj)->child_link_name, childLink);
        SoNode * childNode = getAsInventor(childLink, useScaleFactor);
        EigenTransform jointTransform = getTransform(*pj);
        if (useScaleFactor) scaleTranslation(jointTransform, scaleFactor);

        // ROS_WARN_STREAM("Transform joint "<<(*pj)->name<<": "<<jointTransform);

        allVisuals = addSubNode(childNode, allVisuals, jointTransform);
    }

    return allVisuals;
}


SoNode * Urdf2GraspIt::getAsInventor(const std::string& fromLink, bool useScaleFactor)
{
    Link_Ptr from_link;
    this->robot.getLink(fromLink, from_link);
    return getAsInventor(from_link, useScaleFactor);
}


bool Urdf2GraspIt::writeAsInventor(const std::string& fromLink, const std::string& filename, bool useScaleFactor)
{
    Link_Ptr from_link;
    this->robot.getLink(fromLink, from_link);
    return writeAsInventor(from_link, filename, useScaleFactor);
}

bool Urdf2GraspIt::writeAsInventor(const Link_Ptr& from_link, const std::string& filename, bool useScaleFactor)
{
    SoNode * inv = getAsInventor(from_link, useScaleFactor);
    if (!inv)
    {
        ROS_ERROR("could not generate overall inventor file");
        return false;
    }
    return writeFile(inv, filename);
}


bool Urdf2GraspIt::generateContactsForallVisuals(const std::string& linkName, const int linkNum, const int fingerNum,
        const float coefficient, const std::vector<MarkerSelector::Marker>& markers)
{
    Link_Ptr link = getLink(linkName);

    std::map<std::string, std::vector<Contact_Ptr> >::iterator lCnt = linkContacts.find(linkName);
    if (lCnt == linkContacts.end())
        lCnt = linkContacts.insert(std::make_pair(linkName, std::vector<Contact_Ptr>())).first;

    std::vector<MarkerSelector::Marker>::const_iterator m = markers.begin();
    int visualNum = -1;
    for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        ++visualNum;

        if ((m != markers.end()) && (m->visualNum > visualNum)) continue;

        boost::shared_ptr<urdf::Visual> visual = *vit;

        EigenTransform vTrans = getTransform(visual->origin);   // transform from link reference frame to the visual

        // move forward in markers vector until we get the correct visual number
        while (m != markers.end())
        {
            // ROS_INFO("linkname %s vn %i",m->linkName.c_str(),m->visualNum);
            if (m->visualNum == visualNum)
            {
                break;
            }
            ++m;
        }

        // ROS_INFO("Checking Marker for link %s, visual num %i (m visual num %i)",
        //      linkName.c_str(),visualNum, m->visualNum);
        // if (m==markers.end()) ROS_INFO("No marker in map.");

        // loop over all marker entries we have for this visual number
        while ((m != markers.end()) && (m->visualNum == visualNum))
        {
            Contact_Ptr contact(new Contact());
            contact->linkNum = linkNum;
            contact->fingerNum = fingerNum;

            setUpSoftFrictionEdges(contact->numFrictionEdges, contact->frictionEdges);

            // ROS_INFO("Num friction edges: %i",contact.numFrictionEdges);

            // have to transform from the visual reference frame back to the link reference frame
            Eigen::Vector3d pos(m->coords);
            Eigen::Vector3d norm(m->normal);
            norm.normalize();

            // ROS_INFO_STREAM("Normal: "<<norm);

            // ROS_INFO_STREAM("Visual trans: "<<vTrans);

            EigenTransform toPos = EigenTransform::Identity();
            Eigen::Vector3d dNorm(0, 0, 1);
            // dNorm=vTrans.rotation()*dNorm;
            Eigen::Quaterniond rotNorm = Eigen::Quaterniond::FromTwoVectors(dNorm, norm);
            toPos.translate(pos);
            toPos.rotate(rotNorm);

            EigenTransform jtToPos = vTrans * toPos;
            pos = jtToPos.translation();
            contact->loc = pos;

            Eigen::Quaterniond ori(jtToPos.rotation());
            contact->ori = ori;

            // Contact normal
            norm = dNorm;
            norm.normalize();
            contact->norm = norm;
            contact->cof = coefficient;

            contacts.push_back(contact);
            lCnt->second.push_back(contact);

            // ROS_INFO_STREAM("Adding contact for link "<<linkName<<", visual "<<visualNum<<
            //    ": "<<contact<<" (mark: "<<*m<<") vis trans: "<<vTrans.translation());

            ++m;
        }
    }

    return true;
}




bool Urdf2GraspIt::generateContacts(const std::vector<std::string>& rootFingerJoints, const std::string& palmLinkName,
                                    const float coefficient, const MarkerSelector::MarkerMap& markers)
{
    Link_Ptr palm_link;
    this->robot.getLink(palmLinkName, palm_link);

    // first, do the palm:
    MarkerSelector::MarkerMap::const_iterator palmM = markers.find(palmLinkName);
    if (palmM != markers.end())
    {
        int linkNum = 0;
        int fingerNum = -1;
        if (!generateContactsForallVisuals(palmLinkName, linkNum, fingerNum, coefficient, palmM->second))
        {
            ROS_ERROR("Failed to generate contact for link %s", palmLinkName.c_str());
            return false;
        }
    }

    // now, do all the fingers:
    int fingerNum = -1;
    for (std::vector<std::string>::const_iterator it = rootFingerJoints.begin(); it != rootFingerJoints.end(); ++it)
    {
        ++fingerNum;
        // ROS_INFO("Handling root finger %s",it->c_str());

        Joint_Ptr root_joint = getJoint(*it);
        if (!root_joint.get())
        {
            ROS_ERROR("Could not find joint %s", it->c_str());
            return false;
        }
        std::vector<Joint_Ptr> chain;
        bool onlyActive = true;
        if (!getDependencyOrderedJoints(chain, root_joint, false, onlyActive) || chain.empty())
        {
            ROS_ERROR("Could not get joint chain, joint %s", root_joint->name.c_str());
            return false;
        }

        int linkNum = -1;
        for (std::vector<Joint_Ptr>::iterator cit = chain.begin(); cit != chain.end(); ++cit)
        {
            ++linkNum;
            Joint_Ptr chainJoint = *cit;

            std::string linkName = chainJoint->child_link_name;

            MarkerSelector::MarkerMap::const_iterator markerIt = markers.find(linkName);
            if (markerIt == markers.end())
            {
                // no markers defined for this link
                // ROS_INFO("No markers defined for this link: %s",linkName.c_str());
                continue;
            }


            // ROS_INFO("Marker defined for link %s",linkName.c_str());

            if (!generateContactsForallVisuals(linkName, linkNum, fingerNum, coefficient, markerIt->second))
            {
                ROS_ERROR("Failed to generate contact for link %s", linkName.c_str());
                return false;
            }
        }
    }
    return true;
}


std::string Urdf2GraspIt::getContactsFileContent(const std::string& robotName)
{
    std::stringstream str;

    // Robot name
    str << robotName << std::endl;

    // Number of contacts
    int contSize = contacts.size();
    str << contSize << std::endl;

    for (std::vector<Contact_Ptr>::const_iterator cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
        Contact_Ptr c = *cit;

        str << c->fingerNum << " " << c->linkNum << std::endl;
        str << c->numFrictionEdges << std::endl;

        // The friction edges. One line for each friction edge (total number defined in 2) ).
        //  6 * <number-friction-edges> values.
        for (unsigned int i = 0; i < c->numFrictionEdges; ++i)
        {
            for (unsigned int j = 0; j < 6; ++j)
            {
                str << c->frictionEdges[i * 6 + j] << " ";
            }
            str << std::endl;
        }

        str << static_cast<float>(c->loc.x()) << " "
            << static_cast<float>(c->loc.y()) << " "
            << static_cast<float>(c->loc.z()) << std::endl;

        str << static_cast<float>(c->ori.w()) << " "
            << static_cast<float>(c->ori.x()) << " "
            << static_cast<float>(c->ori.y()) << " "
            << static_cast<float>(c->ori.z()) << std::endl;

        // Frame location (mostly()) equals virtual contact location)
        str << static_cast<float>(c->loc.x()) << " "
            << static_cast<float>(c->loc.y()) << " "
            << static_cast<float>(c->loc.z()) << std::endl;

        str << static_cast<float>(c->norm.x()) << " "
            << static_cast<float>(c->norm.y()) << " "
            << static_cast<float>(c->norm.z()) << std::endl;

        str << static_cast<float>(c->cof) << std::endl;
    }

    return str.str();
}



bool Urdf2GraspIt::generateContactsWithViewer(const std::vector<std::string>& fingerRoots,
        const std::string& palmLinkName, float standard_coefficient)
{
    MarkerSelector markerSelector(0.002);
    markerSelector.init();
    SoNode * ivNode = getAsInventor(palmLinkName, false);
    markerSelector.loadModel(ivNode);
    markerSelector.runViewer();

    MarkerSelector::MarkerMap markers = markerSelector.getMarkers();

    // ROS_INFO("Number of contacts: %lu",markers.size());
    // ROS_INFO("Markers: %s",markerSelector.toString().c_str());

    if (!generateContacts(fingerRoots, palmLinkName, standard_coefficient, markers))
    {
        ROS_ERROR("could not generate contacts");
        return false;
    }
    return true;
}


bool Urdf2GraspIt::loadModelFromXMLString(const std::string& xmlString)
{
    bool success = robot.initString(xmlString);
    if (!success)
    {
        ROS_ERROR("Could not load model from XML string");
        return false;
    }

    outStructure.setRobotName(this->robot.getName());

    return true;
}



bool Urdf2GraspIt::getModelFromFile(const std::string& filename, std::string& xml_string) const
{
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while (xml_file.good())
        {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return true;
    }
    else
    {
        ROS_ERROR("Could not open file [%s] for parsing.", filename.c_str());
    }
    return false;
}


std::string Urdf2GraspIt::getWorldFileTemplate(
    const std::string& robotName,
    const std::vector<DHParam>& dhparams,
    const std::string& prependpath)
{
    return urdf2graspit::xmlfuncs::getWorldFileTemplate(robotName, dhparams, prependpath, negateJointMoves);
}




Urdf2GraspIt::ConversionResultT Urdf2GraspIt::processAll(const std::string& urdfFilename,
        const std::string& palmLinkName,
        const std::vector<std::string>& fingerRootNames,
        const std::string& material)
{
    ConversionResultT result(MESH_OUTPUT_EXTENSION, MESH_OUTPUT_DIRECTORY_NAME);
    result.success = false;

    std::string xml_file;

    if (!getModelFromFile(urdfFilename, xml_file))
    {
        ROS_ERROR("Could not load file");
        return result;
    }


    if (!loadModelFromXMLString(xml_file))
    {
        ROS_ERROR("Could not load file");
        return result;
    }


    result.robotName = this->robot.getName();

    ROS_INFO("Converting files for robot %s", result.robotName.c_str());




    if (!joinFixedLinks(palmLinkName))
    {
        ROS_ERROR("Could not traverse");
        return result;
    }
    // ROS_INFO("00000000000000000000000");
    // p.printModel(palmLinkName);

    Eigen::Vector3d z(0, 0, 1);
    if (!allRotationsToAxis(palmLinkName, z))
    {
        ROS_ERROR("Failed");
        return result;
    }


    float coefficient = 0.2;
    if (!generateContactsWithViewer(fingerRootNames, palmLinkName, coefficient))
    {
        ROS_ERROR("Could not generate contacts");
        return result;
    }


    result = convert(result.robotName, palmLinkName, fingerRootNames, material);
    if (!result.success)
    {
        ROS_ERROR("Could not do the conversion");
        return result;
    }


    result.contacts = getContactsFileContent(result.robotName);

    // ROS_INFO_STREAM("Contacts generated: "<<result.contacts);

    result.success = true;

    return result;
}

