#ifndef URDF2GRASPIT_CONVERTMESH_H
#define URDF2GRASPIT_CONVERTMESH_H

#include <urdf2graspit/Types.h>

namespace urdf_traverser
{
        class UrdfTraverser;
}

namespace urdf2graspit
{
    /**
     * Convert all meshes starting from fromLinkName into the GraspIt! inventor format, and store them in the given
     * mesh files container.
     * While converting, the mesh files can be scaled by the given factor.
     * \param material the material to use in the converted format
     * \param file_extension when the meshes are written to file, this is the extension they will have.
     *      This information may be required for generating the result meshes.
     * \param meshDescXML the resulting GraspIt! XML description files for the meshes, indexed by the link names
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     */
    bool convertGraspItMeshes(
                        urdf_traverser::UrdfTraverser& traverser,
                        const std::string& fromLinkName,
                        double scale_factor,
                        const std::string& material,
                        const std::string& file_extension,
                        const urdf_traverser::EigenTransform& addVisualTransform,
                        std::map<std::string, std::string>& meshDescXML);
}  // namespace

#endif  // URDF2GRASPIT_CONVERTMESH_H
