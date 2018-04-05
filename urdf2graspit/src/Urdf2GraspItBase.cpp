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

#include <urdf_viewer/InventorViewer.h>

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <vector>
#include <set>

using urdf2graspit::Urdf2GraspItBase;

void Urdf2GraspItBase::testVisualizeURDF(const std::string& fromLink)
{
    ROS_INFO("Visualize hand - looks right so far? Close window to continue.");
    bool displayAxes = true;
    float axRad = 0.001;
    float axLen = 0.015;
    Urdf2Inventor::EigenTransform addVisualTrans = Urdf2Inventor::EigenTransform::Identity();
    SoNode * node = getAsInventor(fromLink, false, displayAxes,
                                  axRad, axLen, addVisualTrans, NULL);
    if (!node)
    {
      ROS_ERROR("Could not get inventor node");
    }
    else
    {
      urdf_viewer::InventorViewer view;
      view.init("Test - close me to continue");
      ROS_INFO_STREAM("Model converted, now loading into viewer...");
      view.loadModel(node);
      view.runViewer();
    }
}
    
bool Urdf2GraspItBase::prepareModelForDenavitHartenberg(const std::string& fromLink)
{
    ROS_INFO("### Preparing for DH conversion...");

    ROS_INFO("### Joining fixed links..");
    if (!joinFixedLinks(fromLink))
    {
        ROS_ERROR("Could not join fixed links");
        return false;
    }

    ROS_INFO_STREAM("URDF after joining fixed links: ");
    printModel();
    
    ROS_INFO("### Transforming rotation axes to z...");
    Eigen::Vector3d z(0, 0, 1);
    if (!allRotationsToAxis(fromLink, z))
    {
        ROS_ERROR("Could not transform rotation axes");
        return false;
    }
    dhReadyFrom=fromLink;
    
    // testVisualizeURDF(fromLink);
    
    return true;
}

bool Urdf2GraspItBase::isDHReady(const std::string& fromLink) const
{
    // XXX TODO this should be changed to check dynamically instead of
    // simply using \e dhReadyFrom
    return dhReadyFrom == fromLink;
}
