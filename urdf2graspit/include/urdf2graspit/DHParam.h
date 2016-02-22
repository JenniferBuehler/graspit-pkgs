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

#ifndef URDF2GRASPIT_DHPARAM_H
#define URDF2GRASPIT_DHPARAM_H
// Copyright Jennifer Buehler

#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace urdf2graspit
{

/**
 * \brief Class providing functions for conversion from urdf to a model described by denavit-hartenberg parameters
 * \author Jennifer Buehler
 * \date last edited October 2015
 */
class DHParam
{
public:
    typedef Eigen::Transform<double, 3, Eigen::Affine> EigenTransform;

    DHParam():
        joint(boost::shared_ptr<urdf::Joint>()),
        dof_index(-1),
        d(0),
        r(0),
        theta(0),
        alpha(0) { }
    DHParam(const DHParam& p):
        joint(p.joint),
        dof_index(p.dof_index),
        d(p.d),
        r(p.r),
        theta(p.theta),
        alpha(p.alpha) { }


    boost::shared_ptr<urdf::Joint> joint;
    int dof_index;   // < index of this dh-parameter in the list of DOF specifications of graspit
    double d;  // < distance along previous z
    double r;  // < orthogonal distance from previous z axis to current z
    double theta;  // < rotation of previous x around previous z to current x
    double alpha;  // < rotation of previous z to current z



    DHParam& operator=(const DHParam& p)
    {
        if (&p == this) return *this;
        joint = p.joint;
        dof_index = p.dof_index;
        d = p.d;
        r = p.r;
        theta = p.theta;
        alpha = p.alpha;
        return *this;
    }


    friend std::ostream& operator<<(std::ostream& o, const DHParam& p)
    {
        o << p.joint->name << ": d=" << p.d << ", r=" << p.r << ", theta=" << p.theta
          << ", alpha=" << p.alpha << ", didx=" << p.dof_index;
        return o;
    }

    /**
     * computes denavit hartenberg parameters out of a few world coordinates:
     * - frame (i-1): zi_1, xi_i pi_1 are z axis (rotation axis), x axis (obtained from previous calls to this function for joints earlier in the chain, or any for root joint)
     * and pi_1 is the world position of the frame.
     * - frame i: zi and pi is z-axis and position of frame i.
     * \param xi the new x-axis of frame i will be returned in this paramter.
     */
    static bool toDenavitHartenberg(DHParam& param,
                                    const Eigen::Vector3d& zi_1,
                                    const Eigen::Vector3d& xi_1,
                                    const Eigen::Vector3d& pi_1,
                                    const Eigen::Vector3d& zi,
                                    const Eigen::Vector3d& pi,
                                    Eigen::Vector3d& xi);

    /**
     * Returns the transformation matrix for these DH parameters
     */
    static EigenTransform getTransform(const DHParam& p);




private:
    /**
     * Calcuates DH-Paramters r and alpha between to frames i-1 and i. r is the distance between them along the common normal. alpha is the angle
     * between both z-axises. At the same time, returns the origin of the common normal between frames i-1 and i, and its direction (normalized, but can be re-scaled
     * to length between frames by multiplying with r).
     * \param pi_1 and p_i position in world coordinates of frames i-1 and i
     * \param zi_1 and z_i z-axises of both frames i-1 and i
     */
    static bool getRAndAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                             const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi, double& r, double& alpha,
                             Eigen::Vector3d& commonNormal, Eigen::Vector3d& nOriginOnZi_1);


    /**
     * \param xi not only new x-axis, but also common normal between frames i-1 and i.
     * Its origin is located along zi_1 and given in normOriginOnZi_1, such that xi from there points to frame i
     */
    static bool getDAndTheta(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& xi_1, const Eigen::Vector3d& pi_1,
                             const Eigen::Vector3d& xi, const Eigen::Vector3d& normOriginOnZi_1,
                             double& d, double& theta);


    /**
     * gets the common normal between zi_1 (through pi_1) and zi (through pi).
     * \param nOriginOnZi_1 returns the point on zi_1 which is shortest from zi (this is intersection point if they intersect, or pi_1 if they are parallel)
     * \param shortestDistance the length of the common normal, or also the shortest distance between the lines.
     */
    static bool getCommonNormal(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi, const Eigen::Vector3d& pi_1,
                                const Eigen::Vector3d& pi, Eigen::Vector3d& commonNormal,
                                Eigen::Vector3d& nOriginOnZi_1, double& shortestDistance, bool& parallel);
};

}  //  namespace urdf2graspit
#endif  //  URDF2GRASPIT_DHPARAM_H
