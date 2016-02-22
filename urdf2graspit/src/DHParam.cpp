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
#include <urdf2graspit/DHParam.h>
#include <ros/ros.h>

#define DH_EPSILON 1e-07

using urdf2graspit::DHParam;

template<typename Derived> Eigen::VectorBlock<Derived, 2> subVec2(Eigen::MatrixBase<Derived>& v)
{
    return Eigen::VectorBlock<Derived, 2>(v.derived(), 0);
}

bool equalAxis(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    return (std::fabs(dot - 1.0)) < DH_EPSILON;
}


bool parallelAxis(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    return (std::fabs(dot) - 1.0) < DH_EPSILON;
}


/**
 * returns distance between two lines (squared) L1=pi_1 + m1* zi_1 and L2=pi + m2 * zi.
 * In case the lines are skew (or intersect), parameter paralllel is false, and the
 * function also returns closest points cli_1 and cli (in parallel case, those will be the same points).
 * In case they are parallel, any two closest points along the lines are returned.
 */
double squaredLinesDistance(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                            const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                            bool& parallel, Eigen::Vector3d& cli_1, Eigen::Vector3d& cli)
{
    parallel = false;

    Eigen::Vector3d diff = pi_1 - pi;
    double alpha = -zi_1.dot(zi);
    double bi_1 = diff.dot(zi_1);
    double c = diff.squaredNorm();
    double det = std::fabs(1.0 - alpha * alpha);
    double bi, si_1, si, sqrDist;

    if (det >= DH_EPSILON)
    {
        // Lines are not parallel.
        bi = -diff.dot(zi);
        double invDet = 1.0 / det;
        si_1 = (alpha * bi - bi_1) * invDet;
        si = (alpha * bi_1 - bi) * invDet;
        sqrDist = si_1 * (si_1 + alpha * si + 2.0 * bi_1) +
                  si * (alpha * si_1 + si + 2.0 * bi) + c;
    }
    else
    {
        // Lines are parallel, select any closest pair of points.
        si_1 = -bi_1;
        si = 0.0;
        sqrDist = bi_1 * si_1 + c;
        parallel = true;
    }

    cli_1 = pi_1 + si_1 * zi_1;
    cli = pi + si * zi;

    // Account for numerical round-off errors.
    if (sqrDist < 0.0)
    {
        sqrDist = 0.0;
    }

    return sqrDist;
}

// returns sqrt(squaredLinesDistance(...))
double linesDistance(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                     const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                     bool& parallel, Eigen::Vector3d& cli_1, Eigen::Vector3d& cli)
{
    return sqrt(squaredLinesDistance(zi_1, zi, pi_1, pi, parallel, cli_1, cli));
}


bool DHParam::getRAndAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                           const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi, double& r, double& alpha,
                           Eigen::Vector3d& commonNormal, Eigen::Vector3d& nOriginOnZi_1)
{
    bool parallel = false;
    if (!getCommonNormal(zi_1, zi, pi_1, pi, commonNormal, nOriginOnZi_1, r, parallel))
    {
        ROS_ERROR("Common normal can't be obtained");
        alpha = 0;
        return false;
    }


    if (parallel)
    {
        // ROS_INFO("Parallel case for getRAndAlpha");
        // Re-set nOriginOnZi_1:
        // TODO: Should we consider this case?
        // a) if joint is revolute, set to pi_1 (so that parameter d in the end will be calculated to 0)
        // b) if joint is prismatic, locate it at a reference position?
        nOriginOnZi_1 = pi_1;
        alpha = 0;
        return true;
    }


    alpha = acos(zi_1.dot(zi));
    // maybe adapt sign, as rotation is counter-clockwise around x
    Eigen::AngleAxisd corr(alpha, commonNormal);
    Eigen::Vector3d corrV = corr * zi_1;
    if (!parallelAxis(zi, corrV))
    {
        ROS_ERROR("Consistency: rotation of zi-1 should have aligned with zi. alpha=%f, r=%f", alpha, r);
        ROS_INFO_STREAM(zi << ", " << zi_1 << ", rot to " << corrV << ". common normal: " << commonNormal);
        return false;
    }
    if (!equalAxis(zi, corrV))
    {
        alpha = -alpha;
    }

    return true;
}




bool DHParam::getDAndTheta(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& xi_1, const Eigen::Vector3d& pi_1,
                           const Eigen::Vector3d& xi, const Eigen::Vector3d& normOriginOnZi_1,
                           double& d, double& theta)
{
    Eigen::Vector3d originDiff = normOriginOnZi_1 - pi_1;
    d = originDiff.norm();

    if ((xi_1.norm() < DH_EPSILON) || (xi.norm() < DH_EPSILON))
    {
        ROS_WARN("One of the x-axises is 0, hence theta will be 0");
        theta = 0;
        return true;
    }

    // theta is angle between common normal (xi) and xi_1 around zi_1
    theta = acos(xi_1.dot(xi));
    if (std::fabs(theta) < DH_EPSILON)
    {
        return true;
    }

    // maybe adapt sign, as rotation is counter-clockwise around x
    Eigen::AngleAxisd corr(theta, zi_1);
    Eigen::Vector3d corrV = corr * xi_1;
    if (!parallelAxis(xi, corrV))
    {
        ROS_ERROR("Consistency: rotation of xi-1 should have aligned with xi");
        ROS_INFO_STREAM(xi << ", " << xi_1);
        return false;
    }
    if (!equalAxis(xi, corrV))
    {
        theta = -theta;
    }

    return true;
}

bool DHParam::toDenavitHartenberg(DHParam& param,
                                  const Eigen::Vector3d& zi_1,
                                  const Eigen::Vector3d& xi_1,
                                  const Eigen::Vector3d& pi_1,
                                  const Eigen::Vector3d& zi,
                                  const Eigen::Vector3d& pi,
                                  Eigen::Vector3d& xi)
{
    Eigen::Vector3d nOriginOnZi_1;

    if (!getRAndAlpha(zi_1, zi, pi_1, pi, param.r, param.alpha, xi, nOriginOnZi_1))
    {
        ROS_ERROR("Could not get r and alpha");
        return false;
    }

    if (!getDAndTheta(zi_1, xi_1, pi_1, xi, nOriginOnZi_1, param.d, param.theta))
    {
        ROS_ERROR("Could not get d and theta");
        return false;
    }

    return true;
}





bool DHParam::getCommonNormal(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                              const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                              Eigen::Vector3d& commonNormal, Eigen::Vector3d& nOriginOnZi_1,
                              double& shortestDistance, bool& parallel)
{
    parallel = false;
    Eigen::Vector3d cli_1;
    Eigen::Vector3d cli;

    shortestDistance = linesDistance(zi_1, zi, pi_1, pi, parallel, cli_1, cli);

    nOriginOnZi_1 = cli_1;

    if (shortestDistance < DH_EPSILON)  // lines intersect or are equal
    {
        if ((zi_1 - zi).norm() < DH_EPSILON)
        {
            ROS_WARN_STREAM("z-axises equal and lines intersect. No common normal can be obtained ("
                            << zi_1 << " at " << pi_1 << ", " << zi << " at " << pi << ")");
            commonNormal = Eigen::Vector3d(0, 0, 0);
            return false;
        }
        else
        {
            // ROS_INFO_STREAM("z-axises intersect! "<<zi_1<<" at "<<pi_1<<", "<<zi<<" at "<<pi);
            commonNormal = zi_1.cross(zi);
        }
    }
    else
    {
        commonNormal = cli - cli_1;
        commonNormal.normalize();
        // ROS_INFO_STREAM("commonNormal: "<<commonNormal);
    }

    if (commonNormal.norm() < DH_EPSILON)
    {
        ROS_ERROR("common normal should not be 0 length!");
        return false;
    }

    if (parallel)
    {
        nOriginOnZi_1 = pi_1;
    }

    return true;
}

DHParam::EigenTransform DHParam::getTransform(const DHParam& p)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

    EigenTransform ret;
    ret.setIdentity();

    double ct = cos(p.theta);
    double ca = cos(p.alpha);
    double st = sin(p.theta);
    double sa = sin(p.alpha);
    m(0, 0) = ct;
    m(0, 1) = -st * ca;
    m(0, 2) = st * sa;
    m(0, 3) = p.r * ct;
    m(1, 0) = st;
    m(1, 1) = ct * ca;
    m(1, 2) = -ct * sa;
    m(1, 3) = p.r * st;
    m(2, 0) = 0;
    m(2, 1) = sa;
    m(2, 2) = ca;
    m(2, 3) = p.d;


    ret = EigenTransform(m);

    /*
    //this is how graspit computes the transformation matrix,
    //which actually amounts to the same, as rotating around one axis and
    //then translating along the same is commutative.
    ret.setIdentity();
    Eigen::AngleAxisd arot(p.alpha,Eigen::Vector3d(1,0,0));
    Eigen::AngleAxisd trot(p.theta,Eigen::Vector3d(0,0,1));

    ret.rotate(trot);
    ret.translate(Eigen::Vector3d(0,0,p.d));
    ret.translate(Eigen::Vector3d(p.r,0,0));
    ret.rotate(arot);

    ROS_INFO_STREAM("Transform compares: "<<std::endl<<ret<<std::endl<<EigenTransform(m));
    */
    return ret;
}
