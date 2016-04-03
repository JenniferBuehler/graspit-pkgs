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

#include <urdf2inventor/Helpers.h>

#include <urdf2graspit/Urdf2Graspit.h>
#include <urdf2graspit/XMLFuncs.h>
#include <urdf2graspit/MarkerSelector.h>
#include <urdf2graspit/ContactFunctions.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <vector>
#include <set>

#define RAD_TO_DEG 180/M_PI

using urdf2graspit::Urdf2GraspIt;
using urdf2graspit::markerselector::MarkerSelector;
using urdf2graspit::xmlfuncs::FingerChain;

bool Urdf2GraspIt::getXML(const std::vector<DHParam>& dhparams,
                          const std::vector<std::string>& rootFingerJoints,
                          const std::string& palmLinkName,
                          const std::string * eigenXML,
                          const std::string * contactsVGR,
                          const std::string& mesh_pathprepend, std::string& result)
{
    LinkPtr from_link;
    getRobot().getLink(palmLinkName, from_link);

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
        JointPtr joint = getJoint(*it);
        if (!joint.get())
        {
            ROS_ERROR("Could not find joint %s", it->c_str());
            return false;
        }
        std::vector<JointPtr> chain;
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
        for (std::vector<JointPtr>::iterator cit = chain.begin(); cit != chain.end(); ++cit)
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


void Urdf2GraspIt::getGlobalCoordinates(const JointPtr& joint,
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

bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparameters, const JointPtr& joint,
                               const EigenTransform& parentWorldTransform,
                               const Eigen::Vector3d& parentX, const Eigen::Vector3d& parentZ,
                               const Eigen::Vector3d parentPos, bool asRootJoint)
{
    ROS_INFO_STREAM("Transforming joint " << joint->name << " to DH parameters");

    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
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


    for (std::vector<JointPtr>::const_iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, jointWorldTransform, x, z, pos, false)) return false;
    }

    return true;
}



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparameters, const LinkPtr& from_link)
{
    // first, adjust the transform such that from_link's rotation axis points in z-direction
    JointPtr parentJoint = from_link->parent_joint;
    EigenTransform root_transform;
    root_transform.setIdentity();
    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d pos(0, 0, 0);
    for (std::vector<JointPtr>::const_iterator pj = from_link->child_joints.begin();
            pj != from_link->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, root_transform, x, z, pos, true)) return false;
    }

    return true;
}



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName)
{
    LinkPtr from_link;
    getRobot().getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    return getDHParams(dhparams, from_link);
}

bool Urdf2GraspIt::getDHTransform(const JointPtr& joint, const std::vector<DHParam>& dh, EigenTransform& result)
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
bool Urdf2GraspIt::coordsConvert(const JointPtr& joint, const JointPtr& root_joint,
                                 const std::vector<DHParam>& dh, bool dhToKin, EigenTransform& result)
{
    // starting from the root of a chain, these are the current reference frame transforms in the chain,
    // one in DH space, and one in URDF space
    EigenTransform refFrameDH;
    EigenTransform refFrameURDF;
    refFrameDH.setIdentity();
    refFrameURDF.setIdentity();

    JointPtr iter_joint = root_joint;

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

    JointPtr root_joint;
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
        JointPtr joint = it->joint;
        LinkPtr childLink;
        getRobot().getLink(joint->child_link_name, childLink);

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


bool Urdf2GraspIt::applyTransform(JointPtr& joint, const EigenTransform& trans, bool preMult)
{
    EigenTransform vTrans = getTransform(joint);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, joint);
}

void Urdf2GraspIt::applyTransform(LinkPtr& link, const EigenTransform& trans, bool preMult)
{
    // ROS_INFO("applying transform to link %s",link->name.c_str());

    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        VisualPtr visual = *vit;
        EigenTransform vTrans = getTransform(visual->origin);
        // ROS_INFO_STREAM("a visual for link"<<link->name<<" with transform "<<vTrans);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, visual->origin);
    }


    for (std::vector<CollisionPtr >::iterator cit = link->collision_array.begin();
            cit != link->collision_array.end(); ++cit)
    {
        CollisionPtr coll = *cit;
        EigenTransform vTrans = getTransform(coll->origin);
        if (preMult) vTrans = trans * vTrans;
        else vTrans = vTrans * trans;
        setTransform(vTrans, coll->origin);
    }

    EigenTransform vTrans = getTransform(link->inertial->origin);
    if (preMult) vTrans = trans * vTrans;
    else vTrans = vTrans * trans;
    setTransform(vTrans, link->inertial->origin);

    std::map<std::string, std::vector<ContactPtr> >::iterator lCnt = linkContacts.find(link->name);
    if (lCnt != linkContacts.end())
    {
        for (std::vector<ContactPtr>::iterator it = lCnt->second.begin(); it != lCnt->second.end(); ++it)
        {
            ContactPtr c = *it;
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
    LinkPtr from_link;
    getRobot().getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    for (std::vector<JointPtr>::iterator pj = from_link->child_joints.begin();
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


bool Urdf2GraspIt::allRotationsToAxis(JointPtr& joint, const Eigen::Vector3d& axis)
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);

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
        for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
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
    for (std::vector<JointPtr>::iterator pj = childLink->child_joints.begin();
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




int Urdf2GraspIt::convertGraspItMesh(RecursionParamsPtr& p)
{
    // ROS_INFO("convert mesh for %s",link->name.c_str());
    MeshConvertRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParams>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->link;

    std::string linkMeshFile = link->name + OUTPUT_EXTENSION;
    std::string linkXML = urdf2graspit::xmlfuncs::getLinkDescXML(link, linkMeshFile, param->material);
    // ROS_INFO("XML: %s",linkXML.c_str());

    if (!param->resultMeshes.insert(std::make_pair(link->name, linkXML)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh description file for link %s to the map", link->name.c_str());
        return -1;
    }

    return 1;
}



bool Urdf2GraspIt::convertGraspItMeshes(const std::string& fromLinkName,
                                 double scale_factor, const std::string& material,
                                 std::map<std::string, std::string>& meshDescXML)
{
    LinkPtr from_link;
    getRobot().getLink(fromLinkName, from_link);
    if (!from_link.get())
    {
        ROS_ERROR("Link %s does not exist", fromLinkName.c_str());
        return false;
    }

    // do one call of convertGraspitMesh
    LinkPtr parent;  // leave parent NULL
    MeshConvertRecursionParams::Ptr meshParams(new MeshConvertRecursionParams(parent, from_link, 0,
            scale_factor, material));

    RecursionParamsPtr p(meshParams);
    int cvt = convertGraspItMesh(p);
    if (cvt < 0)
    {
        ROS_ERROR("Could not convert root mesh");
        return false;
    }

    // go through entire tree
    int ret = (cvt == 0) ||
              (this->traverseTreeTopDown(from_link, boost::bind(&Urdf2GraspIt::convertGraspItMesh, this, _1), p) >= 0);

    meshDescXML = meshParams->resultMeshes;

    return ret;
}


void Urdf2GraspIt::scaleContacts(double scale_factor)
{
    if (isContactsScaled) return;
    std::map<std::string, std::vector<ContactPtr> >::iterator linkItr;
    for (linkItr=linkContacts.begin(); linkItr!=linkContacts.end(); ++linkItr)
    {
        for (std::vector<ContactPtr>::iterator it = linkItr->second.begin(); it != linkItr->second.end(); ++it)
        {
            ContactPtr c = *it;
            c->loc *= scale_factor;
        }
    }
    isContactsScaled = true;
}


void Urdf2GraspIt::scaleParams(std::vector<DHParam>& dh, double scale_factor)
{
    if (isDHScaled) return;
    for (std::vector<DHParam>::iterator it = dh.begin(); it != dh.end(); ++it)
    {
        it->d *= scale_factor;
        it->r *= scale_factor;
    }
    isDHScaled = true;
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
    
    // now, scale up all dh parameters to match the scale factor,
    // and also all link/collision/intertial transforms given in the URDF
    scaleParams(dh_parameters, getScaleFactor());
    scaleContacts(getScaleFactor());
    // scale the inventor meshes:
    if (!scale())
    {
        ROS_ERROR("Could not scale up the model");
        return false;
    }

    return true;
}

Urdf2GraspIt::ConversionResultPtr Urdf2GraspIt::preConvert(const ConversionParametersPtr& cparams)
{
   
    GraspItConversionParametersPtr param = architecture_binding_ns::dynamic_pointer_cast<GraspItConversionParameters>(cparams);
    if (!param.get())
    {
        ROS_ERROR("Conversion parameters not of right type");
        return GraspItConversionResultPtr();
    }
    
    ROS_INFO_STREAM("### Urdf2GraspIt::pretConvert for robot "<<param->robotName);
    outStructure.setRobotName(param->robotName); 

    std::string outputMeshDir =  outStructure.getMeshDirPath();
    GraspItConversionResultPtr result(new GraspItConversionResult(OUTPUT_EXTENSION, outputMeshDir));
    result->success = false;
    result->robotName = param->robotName;
   
    ROS_INFO("##### Computing DH parameters out of model");
    if (!dhTransformed && !toDenavitHartenberg(param->rootLinkName))
    {
        ROS_ERROR("Could not transform to DH reference frames");
        return result;
    }

    ROS_INFO("##### scaling DH parameters");
    // scale up all dh parameters to match the scale factor,
    // and also all link/collision/intertial transforms given in the URDF
    scaleParams(dh_parameters, getScaleFactor());
    scaleContacts(getScaleFactor());
    
    return result;
}


Urdf2GraspIt::ConversionResultPtr Urdf2GraspIt::postConvert(const ConversionParametersPtr& cparams, ConversionResultPtr& _result)
{
    GraspItConversionResultPtr result = architecture_binding_ns::dynamic_pointer_cast<GraspItConversionResult>(_result);
    if (!result.get())
    {
        ROS_ERROR("postConvert: result not of right type");
        return GraspItConversionResultPtr();
    }
 
    result->success=false;

    GraspItConversionParametersPtr param = architecture_binding_ns::dynamic_pointer_cast<GraspItConversionParameters>(cparams);
    if (!param.get())
    {
        ROS_ERROR("Conversion parameters not of right type");
        return result;
    }
    
    ROS_INFO_STREAM("### Urdf2GraspIt::postConvert for robot "<<param->robotName);

    if (!convertGraspItMeshes(param->rootLinkName, getScaleFactor(), param->material, result->meshXMLDesc))
    {
        ROS_ERROR("Could not convert meshes");
        return result;
    }

    ROS_INFO("############### Getting XML");

    result->eigenGraspXML = urdf2graspit::xmlfuncs::getEigenGraspXML(dh_parameters, negateJointMoves);

    // path to eigen file from robot directory, needed below. Has to match the one created here.
    std::string eigenXML = outStructure.getEigenGraspFileRel();
    std::string contactsVGR = outStructure.getContactsFileRel();

    if (!getXML(dh_parameters, param->fingerRoots, 
            param->rootLinkName, &eigenXML, &contactsVGR, std::string(), result->robotXML))
    {
        ROS_ERROR("Could not get xml");
        return result;
    }

    // ROS_INFO_STREAM("XML: "<<std::endl<<res.robotXML);

    result->world = getWorldFileTemplate(param->robotName, dh_parameters, outStructure.getRobotFilePath());

    result->success = true; 
    return result;
}

/*Urdf2GraspIt::ConversionResultPtr Urdf2GraspIt::convert(const std::string& robotName,
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

    if (!convertGraspItMeshes(palmLinkName, getScaleFactor(), material, res.meshes, res.meshXMLDesc))
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
}*/

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


int Urdf2GraspIt::addJointLink(RecursionParamsPtr& p)
{
    OrderedJointsRecursionParams::Ptr param = architecture_binding_ns::dynamic_pointer_cast<OrderedJointsRecursionParams>(p);
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


bool Urdf2GraspIt::getDependencyOrderedJoints(std::vector<JointPtr>& result,
        const JointPtr& from_joint, bool allowSplits, bool onlyActive)
{
    LinkPtr childLink;
    getRobot().getLink(from_joint->child_link_name, childLink);
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

bool Urdf2GraspIt::getDependencyOrderedJoints(std::vector<JointPtr>& result, const LinkPtr& from_link,
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

int Urdf2GraspIt::getChildJoint(const JointPtr& joint, JointPtr& child)
{
    LinkPtr childLink = getChildLink(joint);
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

Urdf2GraspIt::LinkPtr Urdf2GraspIt::getChildLink(const JointPtr& joint)
{
    LinkPtr childLink;
    getRobot().getLink(joint->child_link_name, childLink);
    return childLink;
}

Urdf2GraspIt::JointPtr Urdf2GraspIt::getParentJoint(const JointPtr& joint)
{
    LinkConstPtr parentLink = getRobot().getLink(joint->parent_link_name);
    if (!parentLink.get()) return JointPtr();
    return parentLink->parent_joint;
}



bool Urdf2GraspIt::generateContactsForallVisuals(const std::string& linkName, const int linkNum, const int fingerNum,
        const float coefficient, const std::vector<MarkerSelector::Marker>& markers)
{
    LinkPtr link = getLink(linkName);

    std::map<std::string, std::vector<ContactPtr> >::iterator lCnt = linkContacts.find(linkName);
    if (lCnt == linkContacts.end())
        lCnt = linkContacts.insert(std::make_pair(linkName, std::vector<ContactPtr>())).first;

    std::vector<MarkerSelector::Marker>::const_iterator m = markers.begin();
    int visualNum = -1;
    for (std::vector<VisualPtr >::iterator vit = link->visual_array.begin();
            vit != link->visual_array.end(); ++vit)
    {
        ++visualNum;

        if ((m != markers.end()) && (m->visualNum > visualNum)) continue;

        VisualPtr visual = *vit;

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
            ContactPtr contact(new Contact());
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
    LinkPtr palm_link;
    getRobot().getLink(palmLinkName, palm_link);

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

        JointPtr root_joint = getJoint(*it);
        if (!root_joint.get())
        {
            ROS_ERROR("Could not find joint %s", it->c_str());
            return false;
        }
        std::vector<JointPtr> chain;
        bool onlyActive = true;
        if (!getDependencyOrderedJoints(chain, root_joint, false, onlyActive) || chain.empty())
        {
            ROS_ERROR("Could not get joint chain, joint %s", root_joint->name.c_str());
            return false;
        }

        int linkNum = -1;
        for (std::vector<JointPtr>::iterator cit = chain.begin(); cit != chain.end(); ++cit)
        {
            ++linkNum;
            JointPtr chainJoint = *cit;

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

    for (std::vector<ContactPtr>::const_iterator cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
        ContactPtr c = *cit;

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
    bool success = true;
    MarkerSelector markerSelector(0.002);
    markerSelector.init("Marker selector");
    SoNode * node = getAsInventor(palmLinkName,false);
    if (!node)
    {
        ROS_ERROR("Could not get inventor node");
        return false;
    }

    ROS_INFO_STREAM("Model inventor files loaded, now loading into viewer...");
    // XXX TEST for some mesh formats, it does not work if init() is called above,
    // it has to be called here instead. The problem is the call of SoQt::init().
    // If called at all, also subsequent mesh conversions fail. I think it's a problem
    // with ivcon. It was so far found with the .obj meshes of the R2, not with the jaco
    // STL meshes...
    //markerSelector.init("Marker selector");
    markerSelector.loadModel(node);
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


std::string Urdf2GraspIt::getWorldFileTemplate(
    const std::string& robotName,
    const std::vector<DHParam>& dhparams,
    const std::string& prependpath)
{
    return urdf2graspit::xmlfuncs::getWorldFileTemplate(robotName, dhparams, prependpath, negateJointMoves);
}




Urdf2GraspIt::ConversionResultPtr Urdf2GraspIt::processAll(const std::string& urdfFilename,
        const std::string& palmLinkName,
        const std::vector<std::string>& fingerRootNames,
        const std::string& material)
{
    
    std::string outputMeshDir =  outStructure.getMeshDirPath();
    GraspItConversionResultPtr failResult(new GraspItConversionResult(OUTPUT_EXTENSION, outputMeshDir));
    failResult->success = false;

    ROS_INFO_STREAM("### Loading from URDF file "<<urdfFilename<<"...");

    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return failResult;
    }


    ROS_INFO("### Converting files for robot %s", getRobot().getName().c_str());
    ROS_INFO("### Joining fixed links..");

    if (!joinFixedLinks(palmLinkName))
    {
        ROS_ERROR("Could not traverse");
        return failResult;
    }
    // ROS_INFO("00000000000000000000000");
    // p.printModel(palmLinkName);
    
    ROS_INFO("### Transforming rotation axes to z...");

    Eigen::Vector3d z(0, 0, 1);
    if (!allRotationsToAxis(palmLinkName, z))
    {
        ROS_ERROR("Failed");
        return failResult;
    }
    
    ROS_INFO("### Generating contacts...");

    float coefficient = 0.2;
    if (!generateContactsWithViewer(fingerRootNames, palmLinkName, coefficient))
    {
        ROS_ERROR("Could not generate contacts");
        return failResult;
    }
    
    ROS_INFO("### Converting files...");

    ConversionParametersPtr params(new GraspItConversionParameters(getRobot().getName(), palmLinkName, material, fingerRootNames));
    ConversionResultPtr convResult = convert(params);
    if (!convResult->success)
    {
        ROS_ERROR("Could not do the conversion");
        return convResult;
    }

    GraspItConversionResultPtr result = architecture_binding_ns::dynamic_pointer_cast<GraspItConversionResult>(convResult);
    if (!result.get())
    {
        ROS_ERROR("postConvert: result not of right type");
        convResult->success = false;
        return convResult;
    }
 
    result->robotName = getRobot().getName();
    result->contacts = getContactsFileContent(result->robotName);

    // ROS_INFO_STREAM("Contacts generated: "<<result->contacts);

    result->success = true;
    return result;
}

