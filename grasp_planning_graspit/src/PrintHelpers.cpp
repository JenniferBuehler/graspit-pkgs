/**
   Helper ostream operators.

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

#include <grasp_planning_graspit/PrintHelpers.h>

std::ostream& operator<<(std::ostream& o, const Eigen::Vector3f& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}


std::ostream& operator<<(std::ostream& o, const Eigen::Vector3d& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Transform<double, 3, Eigen::Affine>& t)
{
    //  o<<"T: trans="<<t.translation()<<" rot="<<Eigen::Quaterniond(t.rotation());
    Eigen::AngleAxisd ax(t.rotation());
    o << "T: trans = " << Eigen::Vector3d(t.translation()) <<
      " rot = " << ax.angle() << " (angle) / " << ax.axis() << " (axis)";
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Matrix4d& m)
{
    o << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << "," << m(0, 3) << "," <<
      m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << "," << m(1, 3) << "," <<
      m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "," << m(2, 3) << "," <<
      m(3, 0) << "," << m(3, 1) << "," << m(3, 2) << "," << m(3, 3);
    return o;
}

