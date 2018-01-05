/**
   Helper for graspit transforms.

   Copyright (C) 2016 Jennifer Buehler

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
*/

#include <grasp_planning_graspit/GraspItHelpers.h>
#include <graspit/matvec3D.h>

transf GraspIt::getGraspitTransform(const GraspIt::EigenTransform& transform)
{
    Eigen::Vector3d trans = transform.translation();
    Eigen::Quaterniond quat(transform.rotation());

    transf t;
    vec3 translation(trans.x(), trans.y(), trans.z());
    Quaternion quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    t.set(quaternion, translation);

    return t;
}

GraspIt::EigenTransform GraspIt::getEigenTransform(const transf& trans)
{
    GraspIt::EigenTransform transform;
    transform.setIdentity();

    Quaternion q = trans.rotation();
    vec3 t = trans.translation();

    Eigen::Vector3d translation(t.x(), t.y(), t.z());
    Eigen::Quaterniond quaternion(q.w(), q.x(), q.y(), q.z());

    transform = transform.translate(translation);
    transform = transform.rotate(quaternion);

    return transform;
}

