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
#include <urdf_traverser/Types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace urdf2graspit
{

/**
 * \brief Class providing functions for conversion from urdf to a model described by denavit-hartenberg parameters
 *
 * DH Parameters given in this class transform from a reference frame (i) to (i+1).
 * The URDF joint is located at frame (i).
 *
 * \author Jennifer Buehler
 * \date last edited October 2015
 */
class DHParam
{
public:
    typedef urdf_traverser::EigenTransform EigenTransform;
    typedef urdf_traverser::JointConstPtr JointConstPtr;
    typedef urdf_traverser::LinkConstPtr LinkConstPtr;

    DHParam():
        joint(JointConstPtr()),
        dof_index(-1),
        d(0),
        r(0),
        theta(0),
        alpha(0) { }
    DHParam(const DHParam& p):
        joint(p.joint),
        childLink(p.childLink),
        dof_index(p.dof_index),
        d(p.d),
        r(p.r),
        theta(p.theta),
        alpha(p.alpha) { }

    // URDF joint which is located at reference frame (i).
    // The DH parameters transform to frame (i+1).
    JointConstPtr joint;
    LinkConstPtr childLink;
    int dof_index;   // < index of this dh-parameter in the list of DOF specifications of graspit
    double d;  // < distance along previous z
    double r;  // < orthogonal distance from previous z axis to current z
    double theta;  // < rotation of previous x around previous z to current x
    double alpha;  // < rotation of previous z to current z

    DHParam& operator=(const DHParam& p)
    {
        if (&p == this) return *this;
        joint = p.joint;
        childLink = p.childLink;
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
          << ", alpha=" << p.alpha << ", dof_idx=" << p.dof_index;
        if (p.joint->type == urdf::Joint::REVOLUTE)
         o << " (revolute)";
        else if (p.joint->type == urdf::Joint::CONTINUOUS)
         o << " (continuous)";
        else if (p.joint->type == urdf::Joint::PRISMATIC)
         o << " (prismatic)";
        else
         o << " (other type = " << p.joint->type << ")";
        return o;
    }

    /**
     * Computes Denavit Hartenberg parameters.
     * \param zi_1 z axis (rotation axis) in frame i-1
     * \param xi_1 x axis (rotation axis) in frame i-1, obtained
     *      from previous calls to this function for joints earlier in the chain
     *      or any axis for root joint.
     * \param pi_1 world position (global coordinates) in frame i-1, obtained
     *      from previous calls to this function for joints earlier in the chain
     *      (when returned as parameter \e pi),
     *      or for the root joint this is the position of the joint as given
     *      in URDF space (global coordinates).
     * \param zi z-axis of frame i.
     * \param pi position of the joint in frame i.
     *    This will be the global pose of the joint in URDF space.
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


    /**
     * For each child link defined in the DH parameters \e dh the transform
     * the transform from DH to URDF space is returned in \e transforms. The transform is given in the
     * local joint space.
     * \param transforms for each link, the transform from DH to URDF space
     * \retval false if there is a consistency error
     * \param dh2urdf if true, return transforms from DH to URDF. Otherwise, return it the other way round.
     */
    static bool getTransforms(const std::vector<DHParam>& dh, const bool dh2urdf, std::map<std::string,EigenTransform>& transforms);

private:
    /**
     * Calcuates DH-Paramters r and alpha between to frames i-1 and i.
     * \param xi the common normal between frames i-1 and i (must be normalized,
     * so it could be re-scaled to the length between frames by multiplying with r).
     * \param alpha the angle between both z-axises.
     * \param pi_1 and p_i position in world coordinates of frames i-1 and i
     * \param zi_1 and z_i z-axises of both frames i-1 and i
     */
/*
     * \param r the distance between them along the common normal \e xi.
     static bool getRAndAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                             const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi, double& r, double& alpha,
                             Eigen::Vector3d& commonNormal, Eigen::Vector3d& nOriginOnZi_1);*/
      static bool getAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                           const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                           const Eigen::Vector3d& xi,
                           double& alpha);



    /**
     * \param xi not only new x-axis, but also common normal between frames i-1 and i.
     * Its origin is located along zi_1 and given in normOriginOnZi_1, such that xi from there points to frame i
     */
    static bool getDAndTheta(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& xi_1, const Eigen::Vector3d& pi_1,
                             const Eigen::Vector3d& xi, const Eigen::Vector3d& normOriginOnZi_1,
                             double& d, double& theta);


    /**
     * gets the common normal between zi_1 (through pi_1) and zi (through pi).
     * \param nOriginOnZi_1 returns the point on zi_1 which is shortest from zi
     *      (this is intersection point if they intersect, or pi_1 if they are \e parallel)
     * \param shortestDistance the length of the common normal, or also the shortest distance between the lines.
     * \param parallel returns true if zi_1 and zi are parallel
     */
    static bool getCommonNormal(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi, const Eigen::Vector3d& pi_1,
                                const Eigen::Vector3d& pi, Eigen::Vector3d& commonNormal,
                                Eigen::Vector3d& nOriginOnZi_1, double& shortestDistance, bool& parallel);


    static EigenTransform getTransform(const urdf::Pose& p);

    // Get joint transform to parent
    static EigenTransform getTransform(const DHParam::JointConstPtr& joint);

};

}  //  namespace urdf2graspit
#endif  //  URDF2GRASPIT_DHPARAM_H
