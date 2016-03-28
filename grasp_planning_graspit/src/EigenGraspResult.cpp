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

#include <grasp_planning_graspit/PrintHelpers.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <vector>

using GraspIt::EigenGraspResult;

EigenGraspResult::EigenGraspResult(const EigenGraspResult& o):
    relTransform(o.relTransform),
    graspDOFs(o.graspDOFs),
    pregraspDOFs(o.pregraspDOFs),
    egVals(o.egVals),
    legal(o.legal),
    qualEpsilon(o.qualEpsilon),
    qualVolume(o.qualVolume),
    energy(o.energy)

{
}

EigenGraspResult::EigenGraspResult(
    const EigenTransform& _relTransform,
    const std::vector<double>& _graspDOFs,
    const std::vector<double>& _pregraspDOFs,
    const std::vector<double>& _egVals,
    const bool _legal,
    const double _qualEpsilon,
    const double _qualVolume,
    const double _energy):
    relTransform(_relTransform),
    graspDOFs(_graspDOFs),
    pregraspDOFs(_pregraspDOFs),
    egVals(_egVals),
    legal(_legal),
    qualEpsilon(_qualEpsilon),
    qualVolume(_qualVolume),
    energy(_energy)
{
}

/*std::ostream& GraspIt::operator<<(std::ostream& o, const EigenGraspResult& r) {

    GraspIt::EigenGraspResult::EigenTransform relTransform=r.getObjectToHandTransform();
    std::vector<double> graspDOFs = r.getGraspJointDOFs();
    std::vector<double> egVals = r.getEigenGraspValues();

    o<<"Joint DOFs = [";
    for (std::vector<double>::const_iterator it=dofs.begin(); it!=dofs.end(); ++it) {
          o<<*it;
          if ((it+1)!=dofs.end()) o<<", ";
    }
    o<<"] EigenGrasps = [";
    for (std::vector<double>::const_iterator it=egVals.begin(); it!=egVals.end(); ++it) {
          o<<*it;
          if ((it+1)!=egVals.end()) o<<", ";
    }
    o<<"] Relative transform = "<<relTransform;
    return o;
}*/
