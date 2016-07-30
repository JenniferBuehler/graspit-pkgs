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

#ifndef URDF2GRASPIT_CONTACTSGENERATOR_H
#define URDF2GRASPIT_CONTACTSGENERATOR_H
// Copyright Jennifer Buehler

//-----------------------------------------------------
#include <urdf/model.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf2graspit/Urdf2GraspItBase.h>
#include <baselib_binding/SharedPtr.h>
#include <urdf2graspit/DHParam.h>
#include <urdf2graspit/MarkerSelector.h>
#include <urdf2graspit/OutputStructure.h>

// epsilon value to use when comparing whether two axes are the same
// (difference of angle between axes)
#define U2G_EPSILON 1e-07

namespace urdf2graspit
{

/**
 * \brief Provides methods to generate contacts for GraspIt! including an interative viewer.
 * 
 * \date March 2016
 * \author Jennifer Buehler
 */
class ContactsGenerator: public Urdf2GraspItBase
{
public:
    /**
     * \param _scaleFactor the graspit model might have to be scaled (the urdf model is in meters, graspit! in millimeters).
     * This can be specified with this scale factor.
     */
    explicit ContactsGenerator(UrdfTraverserPtr& traverser, float _scaleFactor = 1000):
        Urdf2GraspItBase(traverser, _scaleFactor),
        isContactsScaled(false)
    {
    }

    ~ContactsGenerator()
    {
    }

    /**
     * Generates contacts file out of the markers defined in the map. This function is provided in case the markers information
     * is obtained in a user-defined way. For an interactive marker generation, use function generateContactsWithViewer().
     * After contacts have been generated, they will be put into DH-space specified in \e dh.
     *
     * **Careful**: This function will join fixed links in the loaded URDF, and will also change all rotation axes to be the z axis
     *
     * \param markers a map with the markers.
     * \param coefficient Contact friction coefficient
     */
    bool generateContacts(const std::vector<std::string>& rootFingerJoints, const std::string& palmLinkName,
                          const float coefficient, const markerselector::MarkerSelector::MarkerMap& markers,
                          const std::vector<DHParam>& dh);

    /**
     * Starts a viewer in which the user may select contact points to be defined for the hand. The function generateContacts()
     * is then called so that contacts are defined for the model.
     *
     * \param _displayAxes add the local coordinate system axes of the links to the inventor nodes to display. 
     *      z axis is displayed blue, y axis green, x axis red.
     * \param _axesFromDH if true, then \e _displayAxes will display the DH reference frame axes,
     *      otherwise it will be the link reference frames in URDF.
     * \param _axesRadius radius of the axes, if \e _addAxes is true 
     * \param _axesLength length of the axes, if \e _addAxes is true
     * \param addVisualTransform this transform will be post-multiplied on all links' **visuals** (not links!) local
     *      transform (their "origin"). This can be used to correct transformation errors which may have been 
     *      introduced in converting meshes from one format to the other, losing orientation information
     *      (for example, .dae has an "up vector" definition which may have been ignored)
     * \param facesCCW the faces in the model are ordered counter-clockwise (true should be default)
     */
    bool generateContactsWithViewer(const std::vector<std::string>& fingerRoots,
                                    const std::string& palmLinkName,
                                    float standard_coefficient,
                                    const std::vector<DHParam>& dh,
                                    bool _displayAxes, bool _axesFromDH,
                                    float _axesRadius, float _axesLength,
                                    const EigenTransform& addVisualTransform,
                                    bool facesCCW);

    /**
     * Writes the file for the contacts. Will only work after generateContacts() has been called.
     */
    std::string getContactsFileContent(const std::string& robotName);


protected:
    /**
      * \brief Helper class to encapsulate contact information
      */
    class Contact
    {
    public:
        Contact(): numFrictionEdges(-1), fingerNum(-1), linkNum(-1), cof(0) {}
        Contact(const Contact& o):
            numFrictionEdges(o.numFrictionEdges), frictionEdges(o.frictionEdges),
            fingerNum(o.fingerNum), linkNum(o.linkNum),
            loc(o.loc), ori(o.ori), norm(o.norm), cof(o.cof) {}

        friend std::ostream& operator<<(std::ostream& o, const Contact& c)
        {
            o << c.linkNum << ", " << c.fingerNum << ": " << c.loc;
            return o;
        }

        // Number of friction edges
        unsigned int numFrictionEdges;
        // Friction edges (6*numFrictionEdges doubles)
        // The 6 values specify friction cone boundary wrenches.
        // See more documentation in function ContactsGenerator::setUpFrictionEllipsoid().
        std::vector<double> frictionEdges;
        // Finger number
        int fingerNum;
        // Link number
        int linkNum;
        // Contact (frame) location
        Eigen::Vector3d loc;
        // Contact (frame) orientation
        Eigen::Quaterniond ori;
        // Contact normal (facing away from link)
        Eigen::Vector3d norm;
        // Contact friciton coefficient
        float cof;
    };
    typedef baselib_binding::shared_ptr<Contact>::type ContactPtr;

    bool transformToDHReferenceFrames(const std::vector<DHParam>& dh);
    
    void applyTransformToContacts(LinkPtr& link, const EigenTransform& trans, bool preMult);


    static bool getDHParam(const std::string jointName, const std::vector<DHParam>& dh, DHParam& jointDH);

    SoNode * getAxesAsInventor(const LinkPtr& from_link, 
            const std::vector<DHParam>& dh,
            float _axesRadius, float _axesLength,
            bool linkIsRoot);
private:

    /**
     * scales the contacts by given factor
     */ 
    void scaleContacts(double scale_factor);

    // Helper function for generateContacts()
    bool generateContactsForallVisuals(const std::string& linkName,
                                       const int linkNum, const int fingerNum,
                                       const float coefficient,
                                       const std::vector<markerselector::MarkerSelector::Marker>& markers);
    bool isContactsScaled;
 
    std::vector<ContactPtr> contacts;
    std::map<std::string, std::vector<ContactPtr> > linkContacts;
};

}  //  namespace urdf2graspit
#endif   // URDF2GRASPIT_CONTACTSGENERATOR_H
