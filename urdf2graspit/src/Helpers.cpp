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

#include <urdf2graspit/Helpers.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>


/**
 * ################################################################################################################
 * Minor helpers (e.g. file writing)
 * \author Jennifer Buehler
 * \date last edited October 2015
 * ################################################################################################################
 */

// handle for stdout redirect
int stdout_fd;

void urdf2graspit::helpers::resetStdOut()
{
    if (stdout_fd < 0)
    {
        return;
    }
    fflush(stdout);
    if (dup2(stdout_fd, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("Could not restore stdout");
        return;
    }
    close(stdout_fd);

    // setbuf(stdout,NULL);//reset to unnamed buffer
}

// See http://homepage.ntlworld.com/jonathan.deboynepollard/FGA/redirecting-standard-io.html
void urdf2graspit::helpers::redirectStdOut(const char * toFile)
{
    fflush(stdout);

    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
    int file = open(toFile, O_CREAT | O_APPEND | O_WRONLY, mode);
    if (file < 0)
    {
        ROS_ERROR("could not create new output stream %s: %s", toFile, strerror(errno));
        return;
    }
    stdout_fd = dup(STDOUT_FILENO);                // Clone stdout to a new descriptor
    if (dup2(file, STDOUT_FILENO) < 0)
    {
        ROS_ERROR("could not redirect output stream");
        return;      // Change stdout to file
    }
    // close(file);  // stdout is still valid

    // setvbuf(stdout,toString,_IOFBF,stringSize);
}

void urdf2graspit::helpers::deleteFile(const char* file)
{
    std::remove(file);
}

bool urdf2graspit::helpers::fileExists(const char* file)
{
    return boost::filesystem::exists(file);
}

bool urdf2graspit::helpers::directoryExists(const char* dPath)
{
    // using boost
    return boost::filesystem::exists(dPath);
}

bool urdf2graspit::helpers::makeDirectoryIfNeeded(const char * dPath)
{
    try { 
        boost::filesystem::path dir(dPath);
        boost::filesystem::path buildPath;

        for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end()); it != it_end; ++it){
            buildPath /= *it;
            //std::cout << buildPath << std::endl;
            
            if (!boost::filesystem::exists(buildPath) &&
                !boost::filesystem::create_directory(buildPath))
            {
                ROS_ERROR_STREAM("Could not create directory "<<buildPath);
                return false;
            }
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR_STREAM(ex.what());
        return false;
    }
    return true;
}




bool urdf2graspit::helpers::writeToFile(const std::string& content, const std::string& filename)
{
    std::ofstream outf(filename.c_str());

    if (!outf)
    {
        ROS_ERROR("%s could not be opened for writing!", filename.c_str());
        return false;
    }
    outf << content;
    outf.close();
    return true;
}


// transforms a path specification in the form package://<package-name>/<path> to an absolute path on the computer
std::string urdf2graspit::helpers::packagePathToAbsolute(std::string& packagePath)
{
    // ROS_INFO("We have a mesh %s",packagePath.c_str());
    char pack[1000];
    char rest[1000];
    int numScanned = sscanf(packagePath.c_str(), "package://%[^/]/%s", pack, rest);
    // ROS_INFO("Pack: %s Rest: %s",pack,rest);
    if (numScanned != 2)
    {
        ROS_ERROR("Only package:// style mesh file specifications supported!");
        return std::string();
    }

    std::string packPath = ros::package::getPath(pack);

    if (packPath.empty())  // path, if found, should if it's returned by ros::package
    {
        ROS_ERROR("No package for file specified");
        return std::string();
    }

    std::stringstream absolute;
    absolute << packPath << "/" << rest;

    return absolute.str();
}


std::ostream& operator<<(std::ostream& o, const Eigen::Vector3f& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}


std::ostream& operator<<(std::ostream& o, const Eigen::Vector3d& v)
{
    o << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return o;
}
std::ostream& operator<<(std::ostream& o, const Eigen::Quaterniond& v)
{
    o << "[" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Transform<double, 3, Eigen::Affine>& t)
{
    //  o<<"T: trans="<<t.translation()<<" rot="<<Eigen::Quaterniond(t.rotation());
    Eigen::AngleAxisd ax(t.rotation());
    o << "T: trans=" << Eigen::Vector3d(t.translation()) << " rot=" << ax.angle() << " / " << ax.axis();
    return o;
}

std::ostream& operator<<(std::ostream& o, const Eigen::Matrix4d& m)
{
    o << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << "," << m(0, 3) << "," <<
      m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << "," << m(1, 3) << "," <<
      m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "," << m(2, 3) << "," <<
      m(3, 0) << "," << m(3, 1) << "," << m(3, 2) << "," << m(3, 3);
    return o;
}


std::ostream& operator<<(std::ostream& o, const urdf::Pose& p)
{
    o << "trans=[" << p.position.x << ", " << p.position.y << ", " << p.position.z << "], ";
    o << "rot=[" << p.rotation.x << ", " << p.rotation.y << ", " << p.rotation.z << ", " << p.rotation.w << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const urdf::Vector3& p)
{
    o << "v=[" << p.x << ", " << p.y << ", " << p.z << "], ";
    return o;
}
std::ostream& operator<<(std::ostream& o, const SoSFVec3f& p)
{
    SoSFVec3f d;
    d.copyFrom(p);
    SbString s;
    d.get(s);
    o << "v=[" << s.getString() << "], ";
    return o;
}
std::ostream& operator<<(std::ostream& o, const SoSFRotation& p)
{
    SoSFRotation d;
    d.copyFrom(p);
    SbString s;
    d.get(s);
    o << "v=[" << s.getString() << "], ";
    return o;
}


