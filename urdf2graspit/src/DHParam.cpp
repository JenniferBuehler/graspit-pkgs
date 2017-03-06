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
#include <urdf2inventor/Helpers.h>
#include <urdf2graspit/DHParam.h>
#include <ros/ros.h>

// epsilon for absolute values
// measured in units of the world
#define DH_ZERO_EPSILON 1e-05

// epsilon for a dot product to be considered
// a zero angle
#define DH_ZERO_DOT_EPSILON 1e-03
    
// similar to DH_ZERO_DOT_EPSILON, but
// for the dot product considered as follows:
//   double alpha = -v_1.dot(v);
//   double det = std::fabs(1.0 - alpha * alpha);
// #define DH_ZERO_DET_EPSILON 1e-02

using urdf2graspit::DHParam;

template<typename Derived> Eigen::VectorBlock<Derived, 2> subVec2(Eigen::MatrixBase<Derived>& v)
{
    return Eigen::VectorBlock<Derived, 2>(v.derived(), 0);
}

bool equalAxis(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    return (std::fabs(dot - 1.0)) < DH_ZERO_DOT_EPSILON;
}


bool parallelAxis(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    //ROS_INFO_STREAM("Dot: "<<dot<<", diff: "<<std::fabs((std::fabs(dot) - 1.0)));
    return std::fabs((std::fabs(dot) - 1.0)) < DH_ZERO_DOT_EPSILON;
}

/**
 * \retval 0 neither equal nor parallel
 * \retval 1 parallel
 * \retval 2 equal
 */
int equalOrParallelAxis(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2)
{
    double dot = z1.dot(z2);
    // ROS_INFO_STREAM("Equal or parallel: "<<z1<<" (len "<<z1.norm()<<"), "<<z2<<" (len "<<z2.norm()<<")");
    // ROS_INFO_STREAM("Dot: "<<dot<<", diff: "<<std::fabs((std::fabs(dot) - 1.0))<<", angle="<<acos(dot)*180/M_PI);
    bool parallel = std::fabs((std::fabs(dot) - 1.0)) < DH_ZERO_DOT_EPSILON;
    bool equal = std::fabs(dot - 1.0) < DH_ZERO_DOT_EPSILON;
    if (equal) return 2;
    if (parallel) return 1;
    return 0;
}


bool intersectLinePlane(const Eigen::Vector3d& linePoint, const Eigen::Vector3d& lineDir,
        const Eigen::Vector3d& planePoint, const Eigen::Vector3d& n,
        Eigen::Vector3d& intersect)
{
    if (fabs(lineDir.dot(n)) < DH_ZERO_DOT_EPSILON)
    {
        ROS_ERROR_STREAM("Line "<<lineDir<<" and plane "<<n<<" are parallel");
        return false;
    }    

    // The equation of a plane (points P are on the plane):
    //      n dot (p - planePoint) = 0
    // Which can be expanded to:
    //      n dot p - n dot planePoint = 0
    // and then
    //      n dot p = n dot planePoint
    // The equation of the line (points p on the line along lineDir):
    //      p = linePoint + u * lineDir 
    // The intersection of these two occurs when
    //      n dot (linePoint + u * lineDir) = n dot planePoint
    // expanded:
    //      n dot linePoint + n dot (u*lineDir) = n dot planePoint
    //      n dot (u*lineDir)   = (n dot planePoint) - (n dot linePoint)
    //      n dot (u*lineDir)   = n dot (planePoint - linePoint)
    //      u * (n dot lineDir) = n dot (planePoint - linePoint)
    //      u  = (n dot (planePoint - linePoint)) / (n dot lineDir)

    double u = n.dot(planePoint-linePoint) / n.dot(lineDir);
    intersect = linePoint + lineDir*u;
    return true;
}


/**
 * returns distance between two lines (squared) L1=pi_1 + m1* zi_1 and L2=pi + m2 * zi.
 * In case the lines are skew (or intersect), parameter parallel is false, and the
 * function also returns closest points cli_1 and cli (in intersection case, those will be the same points).
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
    double det2 = std::fabs(1.0 - std::fabs(alpha));
    double bi, si_1, si, sqrDist;

    // ROS_INFO_STREAM("Zi-1 "<<zi_1<<", zi "<<zi<<" alpha: "<<alpha<<" cos: "<<acos(alpha)<<", det "<<det);

    //if (det >= DH_ZERO_DET_EPSILON)
    if (det2 >= DH_ZERO_DOT_EPSILON)
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



#if 1

// returns sqrt(squaredLinesDistance(...))
double linesDistance(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                     const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                     bool& parallel, Eigen::Vector3d& cli_1, Eigen::Vector3d& cli)
{
    return sqrt(squaredLinesDistance(zi_1, zi, pi_1, pi, parallel, cli_1, cli));
}

#else

// another, not yet fully tested, implementation of linesDistance().
double linesDistance(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                     const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                     bool& parallel, Eigen::Vector3d& cli_1, Eigen::Vector3d& cli)
{
    parallel=parallelAxis(zi_1,zi);
    if (parallel)
    {
        ROS_WARN("Not tested yet: parallel z axes");
        cli_1=pi_1;
        cli=pi;
        Eigen::Vector3d pi_conn=pi_1-pi;
        if (pi_conn.norm() < DH_ZERO_EPSILON)
        {
            ROS_INFO("DHParam debug info: pi_1 and pi are equal");
            return 0;
        }
        if (std::fabs(pi_conn.dot(zi_1)) < DH_ZERO_DOT_EPSILON)
        {
            ROS_INFO("DHParam debug info: points are already on closest line");
            return pi_conn.norm();
        }
        Eigen::Vector3d lineDir = zi_1;
        Eigen::Vector3d linePoint = pi_1;
        Eigen::Vector3d point = pi;
        Eigen::Vector3d linePointPlaneN = lineDir.cross(linePoint-point);
        Eigen::Vector3d distLine = linePointPlaneN.cross(lineDir);
        if (!intersectLinePlane(point, distLine, linePoint, distLine, cli_1))
        {
            ROS_ERROR("Could not intersect line and plane");
            return 0;
        }
        cli = point;
    }

    ROS_INFO("Lines are skew");

    Eigen::Vector3d n=zi_1.cross(zi);
    if (std::fabs(n.norm()-1.0) > DH_ZERO_EPSILON)
    {
        ROS_ERROR_STREAM("DHParams: n should be of uniform length! Is "
            <<n.norm()<<" and obtained from "<<zi_1<<" (len "
            <<zi_1.norm()<<"), "<<zi<<"("<<zi.norm()<<"), dot "<<zi_1.dot(zi));
        return 0;
    }

    // plane containing zi_1, parallel to n 
    Eigen::Vector3d zi_1_n = n.cross(zi_1);
    // plane containing zi, parallel to n 
    Eigen::Vector3d zi_n = n.cross(zi);
    
    if (!intersectLinePlane(pi, zi, pi_1, zi_1_n, cli))
    {
        ROS_ERROR("Could not intersect line along zi with plane through zi_1");
        return 0;
    }
    
    if (!intersectLinePlane(pi_1, zi_1, pi, zi_n, cli_1))
    {
        ROS_ERROR("Could not intersect line along zi with plane through zi_1");
        return 0;
    }

    return (cli_1-cli).norm();
}
#endif


#if 0
bool DHParam::getRAndAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                           const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                           double& r, double& alpha,
                           Eigen::Vector3d& commonNormal, Eigen::Vector3d& nOriginOnZi_1)
{
    ROS_INFO_STREAM("getRAndAlpha for "<<zi_1<<", "<<zi);

    bool parallel = false;
    if (!getCommonNormal(zi_1, zi, pi_1, pi, commonNormal, nOriginOnZi_1, r, parallel))
    {
        ROS_ERROR("Common normal can't be obtained");
        alpha = 0;
        return false;
    }

    ROS_INFO_STREAM("Common normal: "<<commonNormal);

    if (std::fabs(zi_1.dot(commonNormal)) > DH_ZERO_DOT_EPSILON)
    {
        ROS_ERROR_STREAM("Consistency: Zi-1 and common normal not orthogonal: "<<zi_1.dot(commonNormal)<<", zi_1 = "<<zi_1<<", commonNormal = "<<commonNormal);
        ROS_INFO_STREAM("angle "<<acos(zi_1.dot(commonNormal))*180/M_PI);
        return false;
    }
    if (std::fabs(zi.dot(commonNormal)) > DH_ZERO_DOT_EPSILON)
    {
        ROS_ERROR_STREAM("Consistency: Zi and common normal not orthogonal: "<<zi.dot(commonNormal)<<", zi = "<<zi<<", commonNormal = "<<commonNormal);
        ROS_INFO_STREAM("angle "<<acos(zi.dot(commonNormal))*180/M_PI);
        return false;
    }

    // ROS_INFO_STREAM("Common normal of "<<zi_1<<" and "<<zi<<": "<<commonNormal<<"(parallel: "<<parallel<<", len="<<commonNormal.norm()<<")");
    
    int zAxEqPl = equalOrParallelAxis(zi_1, zi);
    if ((zAxEqPl > 0) != parallel)
    {
        ROS_ERROR_STREAM("Consistency in DHParams functions: "
            <<"both functions must have considered axes parallel. zi: " 
            <<zi_1<<" zi: "<<zi<<", parallel = "
            <<parallel<<" zAxEqPl = "<<zAxEqPl);
    }

    if (parallel)
    {
        ROS_INFO("DEBUG-INFO DHParam: Parallel case for getRAndAlpha");
        // Re-set nOriginOnZi_1:
        // TODO: Should we consider this case:
        // a) if joint is revolute, set to pi_1 (so that parameter d in the end will be calculated to 0)
        // b) if joint is prismatic, locate it at a reference position?
        nOriginOnZi_1 = pi_1;
        alpha = 0;
        if (zAxEqPl != 2)
        {   // correct alpha to be 
            ROS_INFO_STREAM("DEBUG-INFO DHParam: Correcting alpha for "<<zi<<" as it's not equal to  "<<zi_1);
            alpha = M_PI;
        }
        return true;
    }

    // for alpha, rotation is to be counter-clockwise around x
    alpha = acos(zi_1.dot(zi));
    Eigen::AngleAxisd corr(alpha, commonNormal);
    Eigen::Vector3d corrV = corr * zi_1;
    int corrEqPl = equalOrParallelAxis(zi, corrV);
    if (corrEqPl != 2)
    { 
        // correct alpha so that it causes a counter-clockwise
        // rotation around x
        ROS_INFO_STREAM("DEBUG-INFO DHParams: Correcting alpha (is "
                        <<alpha<<"): "<<zi<<", "<<corrV);
        alpha = -alpha;
    }
    
    if (fabs(r) < 1e-07) r=0;
    if (fabs(alpha) < 1e-07) alpha=0;
    return true;
}
#endif

bool DHParam::getAlpha(const Eigen::Vector3d& zi_1, const Eigen::Vector3d& zi,
                           const Eigen::Vector3d& pi_1, const Eigen::Vector3d& pi,
                           const Eigen::Vector3d& xi, double& alpha)
{
    // ROS_INFO_STREAM("getAlpha for "<<zi_1<<", "<<zi);

    int zAxEqPl = equalOrParallelAxis(zi_1, zi);
    if (zAxEqPl > 0)
    {
        ROS_INFO("DEBUG-INFO DHParam: Parallel case for getAlpha");
        alpha = 0;
        if (zAxEqPl != 2)
        {   // correct alpha to be 
            ROS_INFO_STREAM("DEBUG-INFO DHParam: Correcting alpha for "<<zi<<" as it's not equal to  "<<zi_1);
            alpha = M_PI;
        }
        return true;
    }

    // for alpha, rotation is to be counter-clockwise around x
    alpha = acos(zi_1.dot(zi));
    Eigen::AngleAxisd corr(alpha, xi);
    Eigen::Vector3d corrV = corr * zi_1;
    int corrEqPl = equalOrParallelAxis(zi, corrV);
    if (corrEqPl != 2)
    { 
        // correct alpha so that it causes a counter-clockwise
        // rotation around x
        ROS_INFO_STREAM("DEBUG-INFO DHParams: Correcting alpha (is "
                        <<alpha<<"): "<<zi<<", "<<corrV);
        alpha = -alpha;
    }
    
    if (fabs(alpha) < 1e-07) alpha=0;
    return true;
}


bool DHParam::getDAndTheta(const Eigen::Vector3d& zi_1,
                           const Eigen::Vector3d& xi_1,
                           const Eigen::Vector3d& pi_1,
                           const Eigen::Vector3d& xi,
                           const Eigen::Vector3d& normOriginOnZi_1,
                           double& d, double& theta)
{
    Eigen::Vector3d originDiff = normOriginOnZi_1 - pi_1;
    // ROS_INFO_STREAM("getDAndTheta, difference along z "<<originDiff);
    d = originDiff.norm();
    if (d > DH_ZERO_EPSILON)
    {
      originDiff.normalize();  // normalize for call of equalOrParallelAxis
      int zAxEqPl = equalOrParallelAxis(originDiff, zi_1);
      if (zAxEqPl == 0)
      {
        // neither equal nor parallel
        ROS_ERROR_STREAM("Consistency: translation along z axis should have "
                         <<"been parallel or equal to the z axis!"
                         <<originDiff<<", "<<zi_1<<" (normOriginOnZ="<<normOriginOnZi_1<<")");
        return false;
      }
      else if (zAxEqPl == 1)
      {
        ROS_INFO_STREAM("DEBUG-INFO: Translation along z parallel to z, but not equal, so d is negative");
        d = -d;
      }
    }

    if ((xi_1.norm() < DH_ZERO_EPSILON) || (xi.norm() < DH_ZERO_EPSILON))
    {
        ROS_WARN("One of the x-axises is 0, hence theta will be 0");
        theta = 0;
        return true;
    }

    // theta is angle between common normal (xi) and xi_1 around zi_1
    theta = acos(xi_1.dot(xi));
    if (std::fabs(theta) < DH_ZERO_EPSILON)
    {
        return true;
    }

    // maybe adapt sign, as rotation is counter-clockwise around x
    Eigen::AngleAxisd corr(theta, zi_1);
    Eigen::Vector3d corrV = corr * xi_1;
    int corrEqPl = equalOrParallelAxis(xi, corrV);
    if (corrEqPl != 2)
    {   // correct alpha to be 
        ROS_INFO_STREAM("DEBUG-INFO DHParams: Correcting theta: "<<xi<<", "<<corrV);
        theta = -theta;
    }

    if (fabs(theta) < 1e-07) theta=0;
    if (fabs(d) < 1e-07) d=0;
      
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
    // get the common normal, which is going to be the new x axis
    bool parallel = false;
    Eigen::Vector3d nOriginOnZi_1;
    if (!getCommonNormal(zi_1, zi, pi_1, pi, xi, nOriginOnZi_1, param.r, parallel))
    {
        ROS_ERROR("Common normal can't be obtained");
        return false;
    }

    if (std::fabs(zi_1.dot(xi)) > DH_ZERO_DOT_EPSILON)
    {
        ROS_ERROR_STREAM("Consistency: Zi-1 and common normal not orthogonal: "<<zi_1.dot(xi)<<", zi_1 = "<<zi_1<<", xi = "<<xi);
        ROS_INFO_STREAM("angle "<<acos(zi_1.dot(xi))*180/M_PI);
        return false;
    }
    if (std::fabs(zi.dot(xi)) > DH_ZERO_DOT_EPSILON)
    {
        ROS_ERROR_STREAM("Consistency: Zi and common normal not orthogonal: "<<zi.dot(xi)<<", zi = "<<zi<<", xi = "<<xi);
        ROS_INFO_STREAM("angle "<<acos(zi.dot(xi))*180/M_PI);
        return false;
    }

    // ROS_INFO_STREAM("Common normal of "<<zi_1<<" and "<<zi<<": "<<xi<<"(parallel: "<<parallel<<", closest point="<<nOriginOnZi_1<<")");
   
    // just a consistency check to see that equalOrParallelAxis works the same
    // as the return value of parallel from getCommonNormal(): 
    int zAxEqPl = equalOrParallelAxis(zi_1, zi);
    if ((zAxEqPl > 0) != parallel)
    {
        ROS_ERROR_STREAM("Consistency in DHParams functions: "
            <<"both functions must have considered axes parallel. zi: " 
            <<zi_1<<" zi: "<<zi<<", parallel = "
            <<parallel<<" zAxEqPl = "<<zAxEqPl);
    }

    // the common normal is parallel  
    if (parallel)
    {
        ROS_INFO("DEBUG-INFO DHParam: Parallel case for getCommonNormal");
        // nOriginOnZi_1 should already by pi_1 (set by getCommonNormal).
        // But get this case anyway for debugging.
        // TODO: Should we consider this case:
        // a) if joint is revolute, set to pi_1 (so that parameter d in the end will be calculated to 0)
        // b) if joint is prismatic, locate it at a reference position?
        nOriginOnZi_1 = pi_1;
    }
    
    // cap small values close to 0
    if (fabs(param.r) < 1e-07) param.r=0;

    if (!getAlpha(zi_1, zi, pi_1, pi, xi, param.alpha))
    //if (!getRAndAlpha(zi_1, zi, pi_1, pi, param.r, param.alpha, xi, nOriginOnZi_1))
    {
        ROS_ERROR("Could not get r and alpha");
        return false;
    }

    if (!getDAndTheta(zi_1, xi_1, pi_1, xi, nOriginOnZi_1, param.d, param.theta))
    {
        ROS_ERROR("Could not get d and theta");
        return false;
    }

/*
    // Do the full transform of the joint pose pi_1
    // to pi, this may have changed pi.
    EigenTransform transform = EigenTransform::Identity();
    Eigen::Vector3d z(0,0,1);
    transform.rotate(Eigen::AngleAxisd(param.theta,z));
    transform.translate(z*param.d);
    ROS_INFO_STREAM("..Translated along z by "<<param.d<<": "<<transform * pi_1);
    Eigen::Vector3d x(1,0,0); 
    transform.rotate(Eigen::AngleAxisd(param.alpha,x));
    transform.translate(x*param.r);

    Eigen::Vector3d pi_new = transform * pi_1;
    ROS_INFO_STREAM("PI_NEW: "<<pi_new<<" ( vs URDF space "<<pi<<" )");
    pi = pi_new;

*/


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

    if (shortestDistance < DH_ZERO_EPSILON)
    {   // lines intersect or are equal
        if ((zi_1 - zi).norm() < DH_ZERO_EPSILON)
        {   // z axises are equal
            ROS_WARN_STREAM("z-axises equal. No common normal can be obtained ("
                            << zi_1 << " at " << pi_1 << ", " << zi << " at " << pi << ")");
            commonNormal = Eigen::Vector3d(0, 0, 0);
            return false;
        }
        else
        {   // z-axes intersect
            ROS_INFO_STREAM("DEBUG-INFO: z-axises intersect! "<<zi_1<<" at "<<pi_1<<", "<<zi<<" at "<<pi);
            commonNormal = zi_1.cross(zi);
            // commonNormal.normalize();
        }
        // ROS_INFO_STREAM("Lines are parallel. Shortest distance: "<<shortestDistance<<". Common normal: "<<commonNormal);
    }
    else
    {
        commonNormal = cli - cli_1;
        commonNormal.normalize();
        // ROS_INFO_STREAM("Lines are skew. Shortest distance: "<<shortestDistance<<". Common normal: "<<commonNormal);
    }

    // consistency check
    // ROS_INFO_STREAM("Normal length: "<<commonNormal.norm());
    if (std::fabs(commonNormal.norm()-1.0) > DH_ZERO_EPSILON)
    {
        ROS_ERROR_STREAM("DHParams: common normal should be of uniform length! Is "
            <<commonNormal.norm()<<" and obtained from "<<zi_1<<" (len "
            <<zi_1.norm()<<"), "<<zi<<" (len "<<zi.norm()<<"), dot "<<zi_1.dot(zi));
        return false;
    }

    // normalize, just to smooth out inaccuracies
    commonNormal.normalize();

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

#if 1

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

#else

    //this is how graspit computes the transformation matrix,
    //which actually amounts to the same, as rotating around one axis and
    //then translating along the same is commutative.
    ret.setIdentity();
    Eigen::AngleAxisd trot(p.theta,Eigen::Vector3d(0,0,1));
    ret.rotate(trot);
    ret.translate(Eigen::Vector3d(0,0,p.d));
    Eigen::AngleAxisd arot(p.alpha,Eigen::Vector3d(1,0,0));
    ret.translate(Eigen::Vector3d(p.r,0,0));
    ret.rotate(arot);

    ROS_INFO_STREAM("Transform compares: "<<std::endl<<ret<<std::endl<<EigenTransform(m));
#endif
    return ret;
}



DHParam::EigenTransform DHParam::getTransform(const urdf::Pose& p)
{
    urdf::Vector3 _jtr = p.position;
    Eigen::Vector3d jtr(_jtr.x, _jtr.y, _jtr.z);
    urdf::Rotation _jrot = p.rotation;
    Eigen::Quaterniond jrot(_jrot.w, _jrot.x, _jrot.y, _jrot.z);
    jrot.normalize();
    DHParam::EigenTransform tr;
    tr.setIdentity();
    tr = tr.translate(jtr);
    tr = tr.rotate(jrot);
    return tr;
}

// Get joint transform to parent
DHParam::EigenTransform DHParam::getTransform(const DHParam::JointConstPtr& joint)
{
    return getTransform(joint->parent_to_joint_origin_transform);
}


bool DHParam::getTransforms(const std::vector<DHParam>& dh, const bool dh2urdf,
                            std::map<std::string,EigenTransform>& transforms)
{
    // starting from the root of a chain, these are the current
    // reference frame transforms in the chain,
    // one in DH space, and one in URDF space
    EigenTransform refFrameDH;
    EigenTransform refFrameURDF;
    refFrameDH.setIdentity();
    refFrameURDF.setIdentity();

    JointConstPtr root_joint;
    int chainCnt = -1;
    bool resetChain = false;
    for (std::vector<DHParam>::const_iterator it = dh.begin(); it != dh.end(); ++it)
    {
        if (resetChain)
        {
            refFrameDH.setIdentity();
            refFrameURDF.setIdentity();
            chainCnt = -1;
        }
        resetChain = false;

        ++chainCnt;
        JointConstPtr joint = it->joint;
        LinkConstPtr childLink = it->childLink;
        if (!childLink.get())
        {
            ROS_ERROR("DHParam::urdf2DHTransforms: child link is NULL");
            return false;
        }

        if (childLink->child_joints.empty())
        {
            // this is the end link, and we've defined the end frame to be
            // at the same location as the last joint,
            // so no rotation should be needed?
            resetChain = true;  // reset chain in next iteration
        }

        EigenTransform dhTrans = DHParam::getTransform(*it);

        EigenTransform jointTransform = EigenTransform::Identity();
        if (chainCnt > 0) jointTransform = getTransform(joint);

        // ROS_INFO_STREAM("Dh trans for "<<joint->name<<": "<<dhTrans);
        // ROS_INFO_STREAM("Joint trans for "<<joint->name<<": "<<jointTransform);

        refFrameDH = refFrameDH * dhTrans;
        refFrameURDF = refFrameURDF * jointTransform;

        EigenTransform trans;
        if (dh2urdf)
        {   // dhSpaceToJointSpace
            trans = refFrameDH.inverse() * refFrameURDF;
        }
        else
        {   // jointSpaceToDH 
            trans = refFrameURDF.inverse() * refFrameDH;
        }

        // ROS_INFO_STREAM("Doing transform for joint "<<joint->name<<": "<<dhSpaceToJointSpace);
        if (!transforms.insert(std::make_pair(childLink->name, trans)).second)
        {
            ROS_ERROR_STREAM("Consistency: The link "<<childLink->name<<" was already encountered");
            return false;
        }
    }
    return true;
}

