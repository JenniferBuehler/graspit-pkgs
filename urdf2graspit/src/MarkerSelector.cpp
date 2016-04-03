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

#include <urdf2graspit/MarkerSelector.h>

#include <Inventor/SoDB.h>  // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/SoPickedPoint.h>

#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/details/SoFaceDetail.h>

#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>  // only needed for ROS prints, e.g. ROS_ERROR

using urdf2graspit::markerselector::MarkerSelector;


bool MarkerSelector::writeResults(const std::string& outputFile)
{
    // Now let the user click on the points, and as soon as the window is closed,
    // we save the marker selections:
    if (markers.empty())
    {
        ROS_INFO("No markers selected, so no file written");
        return true;
    }

    ROS_INFO_STREAM("Finished marker selection, now writing marker files to " << outputFile);
    // first, group all markers of all links together
    MarkerMap markerMap = getMarkers();
    std::stringstream str;
    for (MarkerMap::iterator it = markerMap.begin(); it != markerMap.end(); ++it)
    {
        str << it->first << std::endl;
        str << it->second.size() << std::endl;
        for (std::vector<Marker>::iterator m = it->second.begin(); m != it->second.end(); ++m)
        {
            str << m->visualNum << " " << m->coords.x() << " " << m->coords.y() << " " << m->coords.z();
            str << " " << m->normal.x() << " " << m->normal.y() << " " << m->normal.z() << std::endl;
        }
    }

    if (!writeToFile(str.str(), outputFile))
    {
        ROS_ERROR("Could not write to output file.");
        return false;
    }

    return true;
}

MarkerSelector::MarkerMap MarkerSelector::getMarkers()
{
    MarkerMap markerMap;
    for (std::vector<Marker>::iterator it = markers.begin(); it != markers.end(); ++it)
    {
        // std::cout<<*it<<std::endl;
        MarkerMap::iterator mapit = markerMap.find(it->linkName);
        if (mapit == markerMap.end())
            mapit = markerMap.insert(std::make_pair(it->linkName, std::vector<Marker>())).first;
        mapit->second.push_back(*it);
    }
    for (MarkerMap::iterator it = markerMap.begin(); it != markerMap.end(); ++it)
    {
        std::sort(it->second.begin(), it->second.end(), MarkerSelector::sortMarker);
    }
    return markerMap;
}

std::string MarkerSelector::toString()
{
    std::stringstream str;
    for (std::vector<Marker>::iterator it = markers.begin(); it != markers.end(); ++it)
    {
        str << *it << std::endl;
    }
    /*MarkerMap markerMap=getMarkers();
    for (MarkerMap::iterator it=markerMap.begin(); it!=markerMap.end(); ++it) {
        str<<it->first<<std::endl;
        str<<it->second.size()<<std::endl;
        for (std::vector<Marker>::iterator m=it->second.begin(); m!=it->second.end(); ++m) {
            str<<m->visualNum<<" "<<m->x<<" "<<m->y<<" "<<m->z;
            str<<" "<<m->nx<<" "<<m->ny<<" "<<m->nz<<std::endl;
        }
    }*/
    return str.str();
}

bool MarkerSelector::sortMarker(const Marker& i, const Marker& j)
{
    return (i.visualNum < j.visualNum);
}

bool MarkerSelector::writeToFile(const std::string& content, const std::string& filename)
{
    std::ofstream outf(filename.c_str());
    if (!outf)
    {
        ROS_ERROR_STREAM(filename << "could not be opened for writing!");
        return false;
    }
    outf << content;
    outf.close();
    return true;
}


/**
 * string which is used for sscanf to extract following information form an SoNode:
 * the name of the link, and the nubmer of the visual. This sscanf should get first an int (visual number), then a string (link name).
 * it should fail for all nodes except those which contain the visual mesh.
 */
/*#define VISUAL_SCANF "_visual_%i_%s"
SoNode * MarkerSelector::getLinkDesc(const SoPath * path, std::string& linkName, int& visualNum)
{
    for (unsigned int i = path->getLength() - 1; i >= 0;  --i)
    {
        SoNode * n = path->getNode(i);
        std::string name = n->getName().getString();
        // ROS_INFO("Pick path len %s\n",name.c_str());
        char ln[1000];
        int num;
        if (sscanf(name.c_str(), VISUAL_SCANF, &num, ln) < 2) continue;
        // ROS_INFO("num: %i rest: %s\n",num,ln);
        linkName = ln;
        visualNum = num;
        return n;
    }
    return NULL;
}*/


void MarkerSelector::onClickModel(const SoPickedPoint * pPickedPt)
{
    SoPath *pPickPath = pPickedPt->getPath();
    Marker marker;
    SoNode * linkNode = getLinkDesc(pPickPath, marker.linkName, marker.visualNum);
    if (!linkNode)
    {
        ROS_ERROR("Error getting link desc");
        return;
    }
    float x, y, z;
    pPickedPt->getObjectPoint(linkNode).getValue(x, y, z);
    // pPickedPt->getObjectNormal(linkNode).getValue(nx,ny,nz);
    // ROS_INFO("Object point %f %f %f\n",x,y,z);

    marker.setCoords(x, y, z);

    if (!computeCorrectFaceNormal(pPickedPt, isFacesCCW(), marker.normal))
    {
        ROS_INFO("No face normal correction possible, you didn't click on a triangle. Using default normal.");
    }

    // ROS_INFO_STREAM("Adding marker "<<marker);
    addSphere(linkNode, marker.coords, marker_size);
    markers.push_back(marker);
}
