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



void MarkerSelector::init(const char * windowName)
{
    viewWindow = SoQt::init(windowName);
    viewer = new SoQtExaminerViewer(viewWindow);
    root = new SoSelection();
    root->ref();

    SoEventCallback * ecb = new SoEventCallback();
    ecb->addEventCallback(SoMouseButtonEvent::getClassTypeId(), MarkerSelector::mouseBtnCB, this);
    root->addChild(ecb);
}

void MarkerSelector::loadModel(SoNode * model)
{
    if (model) root->addChild(model);
}

bool MarkerSelector::loadModel(const std::string& filename)
{
    SoInput in;
    SoNode  *model = NULL;
    if (!in.openFile(filename.c_str()))
        return false;
    if (!SoDB::read(&in, model) || model == NULL)
        return false;

    root->addChild(model);
    return true;
}

void MarkerSelector::runViewer()
{
    viewer->setSceneGraph(root);
    viewer->show();

    SoQt::show(viewWindow);
    SoQt::mainLoop();
}

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
#define VISUAL_SCANF "_visual_%i_%s"

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
}


void MarkerSelector::addVisualMarker(SoNode * linkNode, const Eigen::Vector3d& pos, float _marker_size)
{
    SoSeparator * link = dynamic_cast<SoSeparator*>(linkNode);
    if (!link)
    {
        ROS_ERROR_STREAM("The link is not a separator, this case should be handeled separately. "
                         << "For now, no markers are displayed.");
        return;
    }

    SoTransform * trans = new SoTransform();
    trans->translation.setValue(pos.x(), pos.y(), pos.z());
    SoSeparator * dot = new SoSeparator();
    dot->addChild(trans);
    SoSphere * s = new SoSphere();
    s->radius = _marker_size;
    SoMaterial * mat = new SoMaterial();
    mat->diffuseColor.setValue(1, 0, 0);
    mat->ambientColor.setValue(0.2, 0.2, 0.2);
    dot->addChild(s);
    link->addChild(mat);
    link->addChild(dot);
}


bool MarkerSelector::computeCorrectFaceNormal(const SoPickedPoint * pick, bool ccw_face, Eigen::Vector3d& normal)
{
    const SoDetail *pickDetail = pick->getDetail();
    if ((pickDetail != NULL) && (pickDetail->getTypeId() == SoFaceDetail::getClassTypeId()))
    {
        // Picked object is a face
        const SoFaceDetail * fd = dynamic_cast<const SoFaceDetail*>(pickDetail);
        if (!fd)
        {
            ROS_ERROR("Could not cast to face detail");
            return false;
        }

        // Find the coordinate node that is used for the faces.
        // Assume that it's the last SoCoordinate3 node traversed
        // before the picked shape.
        SoSearchAction  searchCoords;
        searchCoords.setType(SoCoordinate3::getClassTypeId());
        searchCoords.setInterest(SoSearchAction::LAST);
        searchCoords.apply(pick->getPath());

        if (searchCoords.getPath() == NULL)
        {
            ROS_ERROR("If we picked a face, we need a node which stores the coordinates!");
            return false;
        }

        SoCoordinate3 * coordNode = dynamic_cast<SoCoordinate3*>(searchCoords.getPath()->getTail());
        if (!coordNode)
        {
            ROS_ERROR("Could not cast SoCoordinate3");
            return false;
        }

        if (fd->getNumPoints() != 3)
        {
            ROS_INFO_STREAM("Face with " << fd->getNumPoints() <<
                            " points can't be used for normal calculation, only triangles supported.");
        }

        int p1 = fd->getPoint(0)->getCoordinateIndex();
        SbVec3f coord1 = coordNode->point[p1];
        int p2 = fd->getPoint(1)->getCoordinateIndex();
        SbVec3f coord2 = coordNode->point[p2];
        int p3 = fd->getPoint(2)->getCoordinateIndex();
        SbVec3f coord3 = coordNode->point[p3];

        SbVec3f diff1(coord2.getValue());
        diff1 -= coord1;
        SbVec3f diff2(coord3.getValue());
        diff2 -= coord1;
        SbVec3f cross = diff1.cross(diff2);
        if (!ccw_face) cross = -cross;

        float x, y, z;
        cross.getValue(x, y, z);
        double len = sqrt(x * x + y * y + z * z);
        x /= len;
        y /= len;
        z /= len;

        normal = Eigen::Vector3d(x, y, z);

        return true;
    }
    return false;
}



void MarkerSelector::mouseBtnCB(void *userData, SoEventCallback *pNode)
{
    const SoEvent  *pEvent  = pNode->getEvent();
    MarkerSelector * obj = static_cast<MarkerSelector*>(userData);
    if (!obj)
    {
        ROS_ERROR("Invalid UseData passed into mouseBtnCB");
        return;
    }
    const SoQtViewer *pViewer = obj->viewer;

    if (SoMouseButtonEvent::isButtonPressEvent(pEvent, SoMouseButtonEvent::BUTTON1))
    {
        SoRayPickAction rayPick(pViewer->getViewportRegion());
        rayPick.setPoint(pEvent->getPosition());
        rayPick.setPickAll(false);
        // rayPick.setRadius(1.0);
        rayPick.apply(pViewer->getSceneManager()->getSceneGraph());
        const SoPickedPoint *pPickedPt = rayPick.getPickedPoint();
        if (pPickedPt != NULL)
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

            if (!computeCorrectFaceNormal(pPickedPt, obj->faces_ccw, marker.normal))
            {
                ROS_INFO("No face normal correction possible, you didn't click on a triangle. Using default normal.");
            }

            // ROS_INFO_STREAM("Adding marker "<<marker);
            addVisualMarker(linkNode, marker.coords, obj->marker_size);
            obj->markers.push_back(marker);
        }
    }
}
