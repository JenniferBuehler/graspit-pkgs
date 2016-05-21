#include <ros/ros.h>
#include <urdf_traverser/UrdfTraverser.h>
#include <urdf_traverser/Functions.h>
#include <urdf2inventor/Helpers.h>
#include <urdf2inventor/MeshConvertRecursionParams.h>

#include <urdf2graspit/Types.h>
#include <urdf2graspit/ConvertGraspitMesh.h>
#include <urdf2graspit/XMLFuncs.h>

using urdf_traverser::UrdfTraverser;
using urdf_traverser::RecursionParams;
using urdf2inventor::MeshConvertRecursionParams;

using urdf_traverser::LinkPtr;
using urdf_traverser::JointPtr;
using urdf_traverser::VisualPtr;
using urdf_traverser::GeometryPtr;
using urdf_traverser::MeshPtr;
using urdf_traverser::SpherePtr;
using urdf_traverser::CylinderPtr;
using urdf_traverser::BoxPtr;

/**
 * Function to be called during recursion incurred in convertGraspItMeshes()
 */
int convertGraspItMesh(urdf_traverser::RecursionParamsPtr& p)
{
    // ROS_INFO("convert mesh for %s",link->name.c_str());
    typedef urdf2inventor::MeshConvertRecursionParams<std::string> MeshConvertRecursionParamsT;
    typename MeshConvertRecursionParamsT::Ptr param = baselib_binding_ns::dynamic_pointer_cast<MeshConvertRecursionParamsT>(p);
    if (!param.get())
    {
        ROS_ERROR("Wrong recursion parameter type");
        return -1;
    }

    LinkPtr link = param->getLink();

    std::string linkMeshFile = urdf_traverser::helpers::getFilename(link->name.c_str()) + param->out_extension;
    std::string linkXML = urdf2graspit::xmlfuncs::getLinkDescXML(link, linkMeshFile, param->material);
    // ROS_INFO("XML: %s",linkXML.c_str());

    if (!param->resultMeshes.insert(std::make_pair(link->name, linkXML)).second)
    {
        ROS_ERROR("Could not insert the resulting mesh description file for link %s to the map", link->name.c_str());
        return -1;
    }

    return 1;
}

bool urdf2graspit::convertGraspItMeshes(
                 UrdfTraverser& traverser,
                 const std::string& fromLinkName,
                 double scale_factor,
                 const std::string& material,
                 const std::string& file_extension,
                 const EigenTransform& addVisualTransform,
                 std::map<std::string, std::string>& meshDescXML)
{
    std::string startLinkName=fromLinkName;
    if (startLinkName.empty()){
        startLinkName = traverser.getRootLinkName();
    }
    
    urdf_traverser::LinkPtr startLink = traverser.getLink(startLinkName);
    if (!startLink.get())
    {
        ROS_ERROR("Link %s does not exist", startLinkName.c_str());
        return false;
    }

    typedef urdf2inventor::MeshConvertRecursionParams<std::string> MeshConvertRecursionParamsT;
    typename MeshConvertRecursionParamsT::Ptr meshParams(new MeshConvertRecursionParamsT(
            scale_factor, material, file_extension, addVisualTransform));

    urdf_traverser::RecursionParamsPtr p(meshParams);

    if (traverser.traverseTreeTopDown(startLinkName, boost::bind(&convertGraspItMesh, _1), p, true) <= 0)
    {
        ROS_ERROR_STREAM("Could not convert meshes.");
        return false;
    }

    meshDescXML = meshParams->resultMeshes;
    return true;
}


