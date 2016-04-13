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
// #include <urdf2graspit/MarkerSelector.h>
// #include <urdf2graspit/ContactFunctions.h>
// #include <urdf2graspit/ContactsGenerator.h>

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
                                        Eigen::Vector3d& rotationAxis, Eigen::Vector3d& position) const
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
                               const Eigen::Vector3d parentPos, bool asRootJoint) const
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
        param.joint = readParentJoint(joint);

        if (!param.joint.get())
        {
            ROS_ERROR_STREAM("Consistency: Joint "<<joint->name
                <<" has no parent, should have been added as root joint instead!");
            return false;
        }
        LinkPtr paramChildLink;
        getRobot().getLink(param.joint->child_link_name, paramChildLink);
        if (!paramChildLink.get())
        {
            ROS_ERROR("consistency, no child link");
            return false;
        }
        param.childLink = paramChildLink;
        
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
        param.childLink = childLink;

        param.r = 0;
        param.d = 0;
        param.theta = 0;
        param.alpha = 0;

        dhparameters.push_back(param);
        return true;
    }

    // recurse to child joints
    for (std::vector<JointPtr>::const_iterator pj = childLink->child_joints.begin();
            pj != childLink->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, jointWorldTransform, x, z, pos, false)) return false;
    }

    return true;
}



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparameters, const LinkPtr& from_link) const
{
    if (!isDHReady(from_link->name))
    {
        ROS_ERROR("Need to call prepareModelForDenavitHartenberg() before DH parameters can be calculated");
        return false;
    }
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



bool Urdf2GraspIt::getDHParams(std::vector<DHParam>& dhparams, const std::string& fromLinkName) const
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

bool Urdf2GraspIt::getDHTransform(const JointPtr& joint, const std::vector<DHParam>& dh, EigenTransform& result) const
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
    std::map<std::string,EigenTransform> transforms;
    if (!DHParam::dh2urdfTransforms(dh, transforms))
    {
        ROS_ERROR("Could not get transforms from DH to URDF");
        return false; 
    }

    std::map<std::string,EigenTransform>::iterator it;
    for (it=transforms.begin(); it!=transforms.end(); ++it)
    {
            
        LinkPtr link=getLink(it->first);
        if (!link.get())
        {
            ROS_ERROR("Link %s does not exist", it->first.c_str());
            return false;
        }
        bool preApply = true;
        applyTransform(link, it->second, preApply);
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

    std::string linkMeshFile = urdf2inventor::helpers::getFilename(link->name.c_str()) + OUTPUT_EXTENSION;
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


void Urdf2GraspIt::scaleParams(std::vector<DHParam>& dh, double scale_factor) const
{
    for (std::vector<DHParam>::iterator it = dh.begin(); it != dh.end(); ++it)
    {
        it->d *= scale_factor;
        it->r *= scale_factor;
    }
}


void Urdf2GraspIt::printParams(const std::vector<DHParam>& dh) const
{
    ROS_INFO("--- DH Parameters: ---");
    for (std::vector<DHParam>::const_iterator it = dh.begin(); it != dh.end(); ++it)
    {
        ROS_INFO_STREAM(*it);
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


bool Urdf2GraspIt::checkConversionPrerequisites(const GraspItConversionParametersPtr& param) const
{
    if (!isDHReady(param->rootLinkName))
    {
        ROS_ERROR("Need to call prepareModelForDenavitHartenberg() before DH parameters can be calculated");
        return false;
    }
    
    // Can only convert if there are no active joints between the root
    // joint and the finger bases, and all fixed joints have been joined.
    LinkConstPtr rootLink = readLink(param->rootLinkName);
    if (!rootLink.get())
    {
        ROS_ERROR_STREAM("No link named '"<<param->rootLinkName<<"' found in URDF.");
        return false;
    }
    for (std::vector<std::string>::iterator rIt=param->fingerRoots.begin(); rIt != param->fingerRoots.end(); ++rIt)
    {
        JointConstPtr fingerRootJoint = readJoint(*rIt);
        if (!fingerRootJoint.get())
        {
            ROS_ERROR_STREAM("No link named '"<<*rIt<<"' found in URDF.");
            return false;
        }
        if (!isChildJointOf(rootLink, fingerRootJoint))
        {
            ROS_ERROR_STREAM("Link named '"<<*rIt<<"' is not direct child of root '"<<param->rootLinkName
                <<". This is either the wrong link, or there are other active joints between the root (palm) link and the finger root links."
                <<" This is a requirement for conversion to GraspIt in the current version.");
            return false;
        }
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
    initOutStructure(param->robotName); 
    
    GraspItConversionResultPtr failResult;

    std::string outputMeshDir =  getOutStructure().getMeshDirPath();
    GraspItConversionResultPtr result(new GraspItConversionResult(OUTPUT_EXTENSION, outputMeshDir));
    result->success = false;
    result->robotName = param->robotName;

    if (!checkConversionPrerequisites(param))
    {
        ROS_ERROR("Prerequisites for conversion not fulfilled.");
        return failResult;
    }

    ROS_INFO("##### Computing DH parameters out of model");
    if (!dhTransformed && !toDenavitHartenberg(param->rootLinkName))
    {
        ROS_ERROR("Could not transform to DH reference frames");
        return failResult;
    }

    printParams(dh_parameters);
    
    ROS_INFO("##### Scaling DH parameters");
    // scale up all dh parameters to match the scale factor,
    // and also all link/collision/intertial transforms given in the URDF

    if (!isDHScaled)
    {
        scaleParams(dh_parameters, getScaleFactor());
        isDHScaled = true;
    }

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
    std::string eigenXML = getOutStructure().getEigenGraspFileRel();
    std::string contactsVGR = getOutStructure().getContactsFileRel();

    if (!getXML(dh_parameters, param->fingerRoots, 
            param->rootLinkName, &eigenXML, &contactsVGR, std::string(), result->robotXML))
    {
        ROS_ERROR("Could not get xml");
        return result;
    }

    // ROS_INFO_STREAM("XML: "<<std::endl<<res.robotXML);

    result->world = getWorldFileTemplate(param->robotName, dh_parameters, getOutStructure().getRobotFilePath());

    /*ROS_INFO("### Generating contacts...");
    ContactsGenerator contGen(getScaleFactor());
    // need to load the model into the contacts generator as well.
    // this will then be a fresh model (not the one already converted to DH).
    std::string _robot_urdf=getRobotURDF();
    if (_robot_urdf.empty() || !contGen.loadModelFromXMLString(_robot_urdf))
    {
        ROS_ERROR("Could not load the model into the contacts generator");
        return result;
    }
    float coefficient = 0.2;
    if (!contGen.generateContactsWithViewer(param->fingerRoots, param->rootLinkName, coefficient, dh_parameters))
    {
        ROS_ERROR("Could not generate contacts");
        return result;
    }
    result->contacts = contGen.getContactsFileContent(result->robotName);
    */
    result->success = true; 
    return result;
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
    
    std::string outputMeshDir =  getOutStructure().getMeshDirPath();
    GraspItConversionResultPtr failResult(new GraspItConversionResult(OUTPUT_EXTENSION, outputMeshDir));
    failResult->success = false;

    ROS_INFO_STREAM("### Loading from URDF file "<<urdfFilename<<"...");

    if (!loadModelFromFile(urdfFilename))
    {
        ROS_ERROR("Could not load file");
        return failResult;
    }

    ROS_INFO("### Converting files for robot %s, starting from link %s",
        getRobot().getName().c_str(), palmLinkName.c_str());
    if (!prepareModelForDenavitHartenberg(palmLinkName))
    {
        ROS_ERROR("Could not prepare for DH conversion");
        return failResult;
    }
      
/*    ROS_INFO("### Generating contacts...");
    float coefficient = 0.2;
    if (!generateContactsWithViewer(fingerRootNames, palmLinkName, coefficient))
    {
        ROS_ERROR("Could not generate contacts");
        return failResult;
    }
*/
    ROS_INFO("### Converting files...");
    ConversionParametersPtr params(new GraspItConversionParameters(getRobot().getName(), palmLinkName, material, fingerRootNames));
    ConversionResultPtr convResult = convert(params);
    if (!convResult || !convResult->success)
    {
        ROS_ERROR("Could not do the conversion");
        return convResult ? convResult : failResult;
    }

    GraspItConversionResultPtr result = architecture_binding_ns::dynamic_pointer_cast<GraspItConversionResult>(convResult);
    if (!result.get())
    {
        ROS_ERROR("postConvert: result not of right type");
        return convResult ? convResult : failResult;
    }
 
    result->robotName = getRobot().getName();

    // ROS_INFO_STREAM("Contacts generated: "<<result->contacts);

    result->success = true;
    return result;
}

