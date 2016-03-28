#ifndef GRASP_PLANNING_GRASPIT_EIGENGRASPRESULT_H
#define GRASP_PLANNING_GRASPIT_EIGENGRASPRESULT_H

/**
   Class to store one grasp which results of the grasp planning.

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


#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <grasp_planning_graspit/PrintHelpers.h>

namespace GraspIt
{

/**
 * \brief Stores the values for a grasp.
 *
 * Grasp values consist of
 * - the relative transform from the object to the hand
 * - the joint's DOF values (joint angles)
 * - the Eigengrasp value(s) for the grasp.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class EigenGraspResult
{
public:
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

    EigenGraspResult() {}
    EigenGraspResult(const EigenGraspResult& o);

    /**
     * \param _legal Whether this state is legal or not, usually refers to presence or absence of collisions
     * \param _qualEpsilon The epsilon quality for this grasp
     * \param _qualVolume The volume quality for this grasp
     * \param _energy The energy used by the simulated annealing-based planners
     */
    EigenGraspResult(const EigenTransform& _relTransform,
                     const std::vector<double>& _dofsGrasp,
                     const std::vector<double>& _dofsPregrasp,
                     const std::vector<double>& _egVals,
                     const bool _legal,
                     const double _qualEpsilon,
                     const double _qualVolume,
                     const double _energy);


    const EigenTransform& getObjectToHandTransform() const
    {
        return relTransform;
    }

    /**
     * Returns the joint's DOF values (joint angles) for a closing grasp.
     * The order corresponds to the order the joints were given in the GraspIt
     * robot description file.
     * If generated with urdf2graspit, this corresponds to the order the joints are
     * encountered in a depth-first search of the URDF tree.
     * The size of the returned vector should be the same size as the movable
     * joints used for the EigenGrasp.
     */
    const std::vector<double>& getGraspJointDOFs() const
    {
        return graspDOFs;
    }
    /**
     * Like getGraspJointDOFs(), but for the pre-grasp.
     */
    const std::vector<double>& getPregraspJointDOFs() const
    {
        return pregraspDOFs;
    }

    /**
     * Returns the Eigengrasp value(s) for the grasp.
     */
    const std::vector<double>& getEigenGraspValues() const
    {
        return egVals;
    }

    /**
     * The energy used by the simulated annealing-based planners
     */
    double getEnergy() const
    {
        return energy;
    }

    /**
     *  Whether this state is legal or not, usually refers to presence or absence of collisions
     */
    bool isLegal() const
    {
        return legal;
    }

    /**
     * The epsilon quality for this grasp
     * Jan 16: The quality value is always 0 for the simulated annealing planner. Have to investigate why.
     */
    double qualityEpsilon() const
    {
        return qualEpsilon;
    }


    /**
     * The volume quality for this grasp
     * Jan 16: The quality value is always 0 for the simulated annealing planner. Have to investigate why.
     */
    double qualityVolume() const
    {
        return qualVolume;
    }

    friend std::ostream& operator<<(std::ostream& o, const EigenGraspResult& r)
    {
        if (r.isLegal()) o << "Legal grasp: ";
        else o << "Illegal grasp: ";

        // o<<"Quality (eps) = "<<r.qualEpsilon<<" Quality (vol) = "<<r.qualVolume<<" ";
        o << "Energy = " << r.energy << " Joint DOFs = [";
        for (std::vector<double>::const_iterator it = r.graspDOFs.begin(); it != r.graspDOFs.end(); ++it)
        {
            o << *it;
            if ((it + 1) != r.graspDOFs.end()) o << ", ";
        }
        o << "] EigenGrasps = [";
        for (std::vector<double>::const_iterator it = r.egVals.begin(); it != r.egVals.end(); ++it)
        {
            o << *it;
            if ((it + 1) != r.egVals.end()) o << ", ";
        }
        o << "] Relative transform = " << r.relTransform;
        return o;
    }

private:
    EigenTransform relTransform;

    std::vector<double> graspDOFs;
    std::vector<double> pregraspDOFs;
    std::vector<double> egVals;

    // The energy used by the simulated annealing-based planners
    double energy;

    // Whether this state is legal or not, usually refers to presence or absence of collisions
    bool legal;

    // The epsilon quality for this grasp
    // Jan 16: The quality value is always 0 for the simulated annealing planner. Have to investigate why.
    double qualEpsilon;

    // The volume quality for this grasp
    // Jan 16: The quality value is always 0 for the simulated annealing planner. Have to investigate why.
    double qualVolume;
};
}  // namespace GraspIt
#endif  //  GRASP_PLANNING_GRASPIT_EIGENGRASPRESULT_H
