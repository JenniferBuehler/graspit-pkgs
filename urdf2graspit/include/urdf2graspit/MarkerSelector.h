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

#ifndef URDF2GRASPIT_MARKERSELECTOR_H
#define URDF2GRASPIT_MARKERSELECTOR_H
// Copyright Jennifer Buehler

#include <urdf_viewer/InventorViewer.h>

#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace urdf2graspit
{
namespace markerselector
{
/**
 * \brief Runs a viewer using the Inventor package and allows to select and save markers on the model.
 * \author Jennifer Buehler
 * \date last edited October 2015
 */
class MarkerSelector: public urdf_viewer::InventorViewer
{
public:
    /**
     * \brief represents a marker with x/y/z coordinates and a normal, associated with a link (and visual).
     */
    class Marker
    {
    public:
        Marker(int id): markerID(id) {}
        Marker(const Marker& m):
            linkName(m.linkName),
            visualNum(m.visualNum),
            coords(m.coords),
            normal(m.normal),
            markerID(m.markerID) {}

        ~Marker() {}


        friend std::ostream& operator<<(std::ostream& o, const Marker& m)
        {
            o << m.linkName << "; visual " << m.visualNum << "; Coords " << m.coords
              << "; normal " << m.normal;
            return o;
        }

        void setCoords(const double x, const double y, const double z)
        {
            coords = Eigen::Vector3d(x, y, z);
        }
        void setNormal(const double x, const double y, const double z)
        {
            normal = Eigen::Vector3d(x, y, z);
        }

        // coordinates in link reference frame
        Eigen::Vector3d coords;

        // normal
        Eigen::Vector3d normal;

        // name of the link the marker is on
        std::string linkName;
        // name of the visual the marker is on
        int visualNum;

        // a globally unique ID for this marker
        int markerID;
    };

    typedef std::map<std::string, std::vector<Marker> > MarkerMap;


    /**
     * \param _marker_size the size of the points displayed where marker is put
     * \param _faces_ccw faces are to be treated as counter-clockwise. Needed for normal calculations.
     */
    explicit MarkerSelector(float _marker_size, bool _faces_ccw):
        urdf_viewer::InventorViewer(_faces_ccw),
        marker_size(_marker_size) {}

    MarkerSelector(const MarkerSelector& o):
        marker_size(o.marker_size), markers(o.markers) {}
    ~MarkerSelector() {}

    /**
     * Writes all markers selected while execution runViewer() in the following format, for all
     * links with markers subsequently:
     * link-name {newline}
     * number-of-markers {newline}
     * visual_number x y z nx ny nz{newline} (coordinates and normal, repeated for all number-of-markers markers)
     */
    bool writeResults(const std::string& outputFile);

    /**
     * returns the map of markers which have been generated during running runViewer().
     * the entries for each link name will be ordered by the visual numbers.
     */
    MarkerMap getMarkers();

    std::string toString();

protected:
    virtual void onClickModel(const SoPickedPoint * pickPoint);
    virtual void onMouseBtnClick(SoEventCallback *pNode){}

private:
    // helper function to sort markers
    static bool sortMarker(const Marker& i, const Marker& j);

    static bool writeToFile(const std::string& content, const std::string& filename);
    
    std::vector<Marker> markers;
    
    // map for all the nodes associated to the marker cylinder nodes, sorted by ID
    typedef std::map<int, SoNode*> MarkerNodeMap;

    MarkerNodeMap markerParentNodes;

    // the size of the points displayed where marker is put
    float marker_size;
};

}  //  namespace markerselector
}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_MARKERSELECTOR_H
