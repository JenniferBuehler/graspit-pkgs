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

#include <urdf2graspit/Urdf2GraspItBase.h>
#include <urdf2inventor/Helpers.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <vector>
#include <set>

using urdf2graspit::Urdf2GraspItBase;

    
bool Urdf2GraspItBase::prepareModelForDenavitHartenberg(const std::string& fromLink)
{
    ROS_INFO("### Joining fixed links..");
    if (!joinFixedLinks(fromLink))
    {
        ROS_ERROR("Could not joint fixed links");
        return false;
    }
    // p.printModel(palmLinkName);
    
    ROS_INFO("### Transforming rotation axes to z...");
    Eigen::Vector3d z(0, 0, 1);
    if (!allRotationsToAxis(fromLink, z))
    {
        ROS_ERROR("Could not transform rotation axes");
        return false;
    }
    dhReadyFrom=fromLink;
    return true;
}

bool Urdf2GraspItBase::isDHReady(const std::string& fromLink) const
{
    // XXX TODO this should be changed to check dynamically instead of
    // simply using \e dhReadyFrom
    return dhReadyFrom == fromLink;
}


