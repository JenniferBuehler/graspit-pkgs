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
#include <vector>
#include <math.h>

void setUpFrictionEllipsoid(unsigned int numLatitudes, unsigned int numDirs[], double phi[],
                            double eccen[], unsigned int& numFrictionEdges, std::vector<double>& frictionEdges)
{
    numFrictionEdges = 0;
    for (unsigned int i = 0; i < numLatitudes; i++)
    {
        numFrictionEdges += numDirs[i];
    }

    frictionEdges.resize(6 * numFrictionEdges, 0);

    unsigned int col = 0;
    for (unsigned int i = 0; i < numLatitudes; i++)
    {
        double cosphi = cos(phi[i]);
        double sinphi = sin(phi[i]);
        for (unsigned int j = 0; j < numDirs[i]; j++)
        {
            double theta = j * 2 * M_PI / numDirs[i];

            double num = cos(theta) * cosphi;
            double denom = num * num / (eccen[0] * eccen[0]);
            num = sin(theta) * cosphi;
            denom += num * num / (eccen[1] * eccen[1]);
            num = sinphi;
            denom += num * num / (eccen[2] * eccen[2]);
            denom = sqrt(denom);

            frictionEdges[col * 6]   = cos(theta) * cosphi / denom;
            frictionEdges[col * 6 + 1] = sin(theta) * cosphi / denom;
            frictionEdges[col * 6 + 2] = 0;
            frictionEdges[col * 6 + 3] = 0;
            frictionEdges[col * 6 + 4] = 0;
            frictionEdges[col * 6 + 5] = sinphi / denom;
            col++;
        }
    }
}

void setUpSoftFrictionEdges(unsigned int& numEdges, std::vector<double>& frictionEdges)
{
    // ROS_INFO("Setting up SOFT contact friction edges");

    double eccen[3];

    // magnitude of tangential friction:
    // in the original graspit code, this is influenced by the collision with another body
    // to obtain the torque as well. This is skipped in this implementation here.
    eccen[0] = 1;
    eccen[1] = 1;
    eccen[2] = 1;


    // XXX check original SoftContact::setUpFrictionEdges() for other stuff done
    // with the eccen field which could be worth doing for this purpose here as well

    // various possible approximations for the friction ellipsoid

    /*
    int numDirs[9] = {1,3,5,7,8,7,5,3,1};
    double phi[9] = {M_PI_2, M_PI_2*0.6, M_PI_2*0.3, M_PI_2*0.15, 0.0,
                     -M_PI_2*0.15, -M_PI_2*0.3, -M_PI_2*0.6, -M_PI_2};
    return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
    */

    /*
    int numDirs[9] = {1,8,8,8,8,8,8,8,1};
    double phi[9] = {M_PI_2, M_PI_2*0.66, M_PI_2*0.33, M_PI_2*0.165, 0.0,
                  -M_PI_2*0.165, -M_PI_2*0.33, -M_PI_2*0.66, -M_PI_2};
    return Contact::setUpFrictionEdges(9,numDirs,phi,eccen);
    */

    unsigned int numDirs[5] = {1, 5, 8, 5, 1};
    double phi[5] = {M_PI_2, M_PI_2 * 0.50, 0.0, -M_PI_2 * 0.50, -M_PI_2};
    return setUpFrictionEllipsoid(5, numDirs, phi, eccen , numEdges, frictionEdges);
}


void setUpFrictionEdges(unsigned int& numEdges, std::vector<double>& frictionEdges)
{
    double eccen[3] = {1, 1, 1};
    unsigned int numDirs[1] = {8};
    double phi[1] = {0.0};
    setUpFrictionEllipsoid(1, numDirs, phi, eccen, numEdges, frictionEdges);
}
