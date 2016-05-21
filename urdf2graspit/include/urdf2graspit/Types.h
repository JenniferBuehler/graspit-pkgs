#ifndef URDF2GRASPIT_TYPES_H
#define URDF2GRASPIT_TYPES_H

#include <urdf_traverser/Types.h>

namespace urdf2graspit
{
    typedef urdf_traverser::EigenTransform EigenTransform;
    typedef urdf_traverser::LinkPtr LinkPtr;
    typedef urdf_traverser::LinkConstPtr LinkConstPtr;
    typedef urdf_traverser::JointPtr JointPtr;
    typedef urdf_traverser::JointConstPtr JointConstPtr;
    typedef urdf_traverser::InertialPtr InertialPtr;
    typedef urdf_traverser::VisualPtr VisualPtr;
    // typedef urdf_traverser::GeometryPtr GeometryPtr;
    typedef urdf_traverser::MeshPtr MeshPtr;
}  // namespace

#endif  // URDF2GRASPIT_TYPES_H
