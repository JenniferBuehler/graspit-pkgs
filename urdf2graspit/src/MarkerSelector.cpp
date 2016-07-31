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
#include <urdf2inventor/IVHelpers.h>

#include <Inventor/SoDB.h>  // for file reading
#include <Inventor/SoInput.h>   // for file reading
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/SoPickedPoint.h>

#include <Inventor/nodes/SoMatrixTransform.h>
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
 * Gets the transform along the path from \e fromIdx to \e toIdx
 */
bool getTransform(const SoPath* p, unsigned int fromIdx, unsigned int toIdx, urdf2inventor::EigenTransform& transform)
{
    SbMatrix sbTransform;
    sbTransform.makeIdentity();

    if ((toIdx >= p->getLength()) || (toIdx <0))
    {
        ROS_ERROR_STREAM("Cannot compute transform for end index out of bounds ("<<toIdx<<")");
        return false;
    }
    if ((fromIdx >= p->getLength()) || (fromIdx <0))
    {
        ROS_ERROR_STREAM("Cannot compute transform for start index out of bounds ("<<fromIdx<<")");
        return false;
    }

    for (int i = fromIdx; i <= toIdx;  ++i)
    {
        SoNode * n = p->getNode(i);
        std::string name = n->getName().getString();
        //ROS_INFO("Path[%i]: %s, type %s",i,name.c_str(),n->getTypeId().getName().getString());
        SoSeparator * nSep = dynamic_cast<SoSeparator*>(n);
        if (!nSep) continue; // only separators may have children which are transforms
        for (int c=0; c<nSep->getNumChildren(); ++c)
        {
            SoNode * child = nSep->getChild(c);
            if (!child) continue;
            //ROS_INFO_STREAM("Child: "<<child->getTypeId().getName().getString());
            SoTransformation * trNode = dynamic_cast<SoTransformation*>(child);
            if (trNode)
            {
                SoTransform * tNode = dynamic_cast<SoTransform*>(child);
                if (!tNode)
                {
                    ROS_ERROR_STREAM("Transformation node was found in MarkerSelector::getTransform() "
                            <<"(type "<<child->getTypeId().getName().getString()
                            <<") which still needs to be implemented (line "<<__LINE__<<")");
                    continue;
                }
                SbMatrix tmpMat;
                
                /*float x,y,z,w;
                tNode->center.getValue().getValue(x,y,z);
                ROS_INFO_STREAM("Center " <<x<<","<<y<<","<<z);
                tNode->scaleFactor.getValue().getValue(x,y,z);
                ROS_INFO_STREAM("ScaleFactor " <<x<<","<<y<<","<<z);
                tNode->scaleOrientation.getValue().getValue(x,y,z,w);
                ROS_INFO_STREAM("ScaleOri " <<x<<","<<y<<","<<z<<","<<w);
                tNode->rotation.getValue().getValue(x,y,z,w);
                ROS_INFO_STREAM("Rot " <<x<<","<<y<<","<<z<<","<<w);
                tmpMat.makeIdentity();
                tmpMat.setTranslate(tNode->translation.getValue());
                sbTransform.multRight(tmpMat);
                tmpMat.makeIdentity();
                tmpMat.setRotate(tNode->rotation.getValue());
                sbTransform.multRight(tmpMat);*/

                tmpMat.setTransform(tNode->translation.getValue(), tNode->rotation.getValue(), 
                        tNode->scaleFactor.getValue());//, tNode->scaleOrientation.getValue(), tNode->center.getValue());
                sbTransform.multRight(tmpMat);
            }
        }
    }
    
    urdf2inventor::EigenTransform egTransform=urdf2inventor::getEigenTransform(sbTransform);

    // for some reason I haven't gotten to the bottom of yet, the matrix is *not* the same when
    // we build it again from scratch, even if there is no shearing in the original sbTransform.
    // i.e. even if *only* a rotation was explicitly set for sbTransform above, and we do an
    // additional  
    //      egTransform=urdf2inventor::EigenTransform(egTransform.rotation());
    // the matrix vary still! For now, work around this by only using translation and rotation.
    // XXX TODO this has to be done properly at some stage.
    //ROS_INFO_STREAM(std::endl<<urdf2inventor::printMatrix(egTransform));
    transform.setIdentity();
    transform.translate(egTransform.translation());
    transform=transform*urdf2inventor::EigenTransform(egTransform.rotation());
    //ROS_INFO_STREAM(std::endl<<urdf2inventor::printMatrix(transform));
    return true;
}


/**
 * string which is used for sscanf to extract following information form an SoNode:
 * the name of the link, and the nubmer of the visual. This sscanf should get first an int (visual number), then a string (link name).
 * it should fail for all nodes except those which contain the visual mesh.
 */
#define VISUAL_SCANF "_visual_%i_%s"

/**
 * string used for sscanf to find a marker along with its number
 */
#define MARKER_SCANF "contact_marker_%i_%s"


void MarkerSelector::onClickModel(const SoPickedPoint * pPickedPt)
{
    SoPath *pPickPath = pPickedPt->getPath();

    // First, see if a marker was clidked.
    int mId, mPos;
    std::string mLinkName;
    SoNode * markerNode = getIntStr(MARKER_SCANF, pPickPath, mLinkName, mId, mPos);
    if (markerNode)
    {
        ROS_INFO_STREAM("Marker "<<mId<<", name "<<mLinkName<<" clicked! Removing...");
        MarkerNodeMap::iterator mpIt=markerParentNodes.find(mId);
        if (mpIt==markerParentNodes.end())
        {
            ROS_ERROR("Marker was not found in the existing map, it should have been!");
            return;
        }
        bool markerRemoved=false;
        for (std::vector<Marker>::iterator mIt=markers.begin(); mIt!=markers.end(); ++mIt)
        {
            if (mIt->markerID==mId)
            {
                markerRemoved=true;
                markers.erase(mIt);
                break;
            }
        }
        if (!markerRemoved)
        {
            ROS_ERROR("Could not find marker in the list. Will not remove it.");
            return;
        }
        SoSeparator * _mSep = dynamic_cast<SoSeparator*>(mpIt->second);
        if (!_mSep)
        {
            ROS_ERROR("Marker parent node is not a separator, can't remove it");
            return;
        }
        _mSep->removeChild(markerNode);
        return;
    }

    mId=markerParentNodes.size();
    Marker marker(mId);
    int linkIdx;
    SoNode * linkNode = getIntStr(VISUAL_SCANF, pPickPath, marker.linkName, marker.visualNum, linkIdx);
    if (!linkNode)
    {
        ROS_INFO("No link or marker was clicked.");
        return;
    }

    float x, y, z;
    pPickedPt->getObjectPoint(linkNode).getValue(x, y, z);
    // pPickedPt->getObjectNormal(linkNode).getValue(nx,ny,nz);
    ROS_INFO_STREAM("Clicked link "<<marker.linkName<<", point "<<","<<x<<","<<y<<","<<z);

    marker.setCoords(x, y, z);

    int shapeIdx;
    if (!computeCorrectFaceNormal(pPickedPt, isFacesCCW(), marker.normal, shapeIdx))
    {
        ROS_WARN("No face normal correction possible. Using default normal.");
    }
    

    // get the transform from the link to the shape form: we have to consider this transform for the normal as well. 
    // ROS_INFO_STREAM("Index of link in path: "<<linkIdx<<" / "<<(pPickPath->getLength()-1)<<" and shape in path: "<<shapeIdx);
    urdf2inventor::EigenTransform link2VertexTransform;
    if (!getTransform(pPickPath, linkIdx, shapeIdx, link2VertexTransform))
    {
        ROS_WARN("Cannot get tranform between link and geometry node, normals may be off.");
    }
        
    SoSeparator * _nodeSep = dynamic_cast<SoSeparator*>(linkNode);
    if (!_nodeSep)
    {
        ROS_WARN_STREAM("The node for link "<<marker.linkName
            <<" is not a separator, so cannot add visual marker sphere");
    }
    else 
    {
        // ROS_INFO_STREAM("Adding marker "<<marker);
        //urdf2inventor::addSphere(_nodeSep, marker.coords, marker_size, 1, 0, 0);
        // compute rotation from z to normal
        Eigen::Vector3d zAxis(0,0,1);
        urdf2inventor::EigenTransform toZ(Eigen::Quaterniond::FromTwoVectors(zAxis,marker.normal));
        urdf2inventor::EigenTransform cylTrans;
        cylTrans.setIdentity();
        cylTrans.translate(marker.coords);
        cylTrans=cylTrans*link2VertexTransform*toZ;
        float radius = marker_size;
        float height = radius*4;
        std::stringstream str;
        str<<"contact_marker_"<<mId<<"_"<<marker.linkName;
        ROS_INFO_STREAM("Adding a new marker named "<<str.str());
        urdf2inventor::addCylinder(_nodeSep, cylTrans, radius, height, 1, 0, 0, 0, str.str().c_str());
        markerParentNodes.insert(std::make_pair(mId,_nodeSep));
    }

    marker.normal = link2VertexTransform.rotation() * marker.normal;
    markers.push_back(marker);
}
