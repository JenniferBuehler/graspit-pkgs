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

#ifndef URDF2GRASPIT_CONTACTFUNCTIONS_H
#define URDF2GRASPIT_CONTACTFUNCTIONS_H

#include <vector>

/**
 * Copied and adapted from GraspIt! Contact class, April 2014.
 *
 * Sets up the friction edges of this contact using an ellipsoid
 * approximation. This is convenience function, as friction edges can
 * be set in many ways. However, we currently use PCWF and SFC models
 * which are both cases of linearized ellipsoids, so this function
 * can be used for both.
 *
 * Consider a 3D friction ellipsoid, where the first two dimensions are
 * tangential frictional force (along X and Y) and the third is frictional
 * torque (along Z). This function samples this ellipsoid at \a numLatitudes
 * latitudes contained in \a phi[]; at each latitude l it takes \a numDirs[l]
 * equally spaced discrete samples. Each of those samples becomes a friction
 * edge, after it is converted to the full 6D space by filling in the other
 * dimensions with zeroes.
 *
 * \param numFrictionEdges output: number of edges generated
 * \param frictionEdges the array (size numEdges*6) defining the friction cone
 */
extern void setUpFrictionEllipsoid(unsigned int numLatitudes, unsigned int numDirs[], double phi[],
                            double eccen[], unsigned int& numFrictionEdges, std::vector<double>& frictionEdges);



/**
 * Copied and adapted from GraspIt! Contact class (soft contact), April 2014.
 *
 * Sets up friction edges as a 3D friction ellipsoid. All the computations for
 * fitting analytical surfaces to the two bodies should already have been
 * completed.
 */
extern void setUpSoftFrictionEdges(unsigned int& numEdges, std::vector<double>& frictionEdges);


/*
 * Copied and adapted from GraspIt! PointContact class, April 2014.
 *
 * Set up friction as a linearized circle; a PCWF can only have friction
 * forces (no torques) in the tangential plane of the contact. When normal
 * force will be added later, the friction circle becomes the more
 * familiar contact cone.
 *
 * \param numEdges output: number of edges generated
 * \param frictionEdges the array (size numEdges*6) defining the friction cone
 */
extern void setUpFrictionEdges(unsigned int& numEdges, std::vector<double>& frictionEdges);




#endif  // URDF2GRASPIT_CONTACTFUNCTIONS_H
