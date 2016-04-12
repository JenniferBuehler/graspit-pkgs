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

#include <ros/ros.h>

#include <urdf2inventor/Helpers.h>
#include <urdf2graspit/Urdf2Graspit.h>
#include <urdf2graspit/ContactsGenerator.h>
#include <urdf2graspit/FileIO.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urdf2graspit", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    if (argc < 6)
    {
        ROS_ERROR("Not enough arguments!");
        ROS_INFO_STREAM("Usage: " << argv[0] <<
                        " <urdf-file> <output-directory> <palm link name>" <<
                        " <finger joint name 0> ... <finger joint name n>");
        return 0;
    }

    // set parameters

    std::string urdf_filename = std::string(argv[1]);
    ROS_INFO("URDF file: %s", urdf_filename.c_str());

    std::string outputDir = std::string(argv[2]);
    ROS_INFO("Output dir: %s", outputDir.c_str());

    std::string palmLinkName = std::string(argv[3]);
    ROS_INFO("Hand root: %s", palmLinkName.c_str());

    std::vector<std::string> roots;
    for (unsigned int i = 4; i < argc; ++i)
    {
        std::string fingerJoint = std::string(argv[i]);
        ROS_INFO("Joint: %s", fingerJoint.c_str());
        roots.push_back(fingerJoint);
    }

    std::string outputMaterial = "plastic";
    double scaleFactor = 1000;
    priv.param<double>("scale_factor", scaleFactor, scaleFactor);
    ROS_INFO("scale_factor: <%f>", scaleFactor);

    ROS_INFO("### Getting DH parameters...");
    urdf2graspit::Urdf2GraspIt converter(scaleFactor);
    if (!converter.loadModelFromFile(urdf_filename))
    {
        ROS_ERROR("Could not load the model into the contacts generator");
        return 0;
    }

    if (!converter.prepareModelForDenavitHartenberg(palmLinkName))
    {
        ROS_ERROR("Could not prepare model for DH parameters");
        return 0;
    }
    std::vector<urdf2graspit::DHParam> dh_parameters;
    if (!converter.getDHParams(dh_parameters, palmLinkName))
    {
        ROS_ERROR("Could not retrieve DH parameters from model");
        return 0;
    }

    ROS_INFO("### Generating contacts for robot %s...", converter.getRobotName().c_str());
    urdf2graspit::ContactsGenerator contGen(scaleFactor);
    if (!contGen.loadModelFromFile(urdf_filename))
    {
        ROS_ERROR("Could not load the model into the contacts generator");
        return 0;
    }
    float coefficient = 0.2;
    if (!contGen.generateContactsWithViewer(roots, palmLinkName, coefficient, dh_parameters))
    {
        ROS_ERROR("Could not generate contacts");
        return 0;
    }
    std::string contacts = contGen.getContactsFileContent(contGen.getRobotName());
    ROS_INFO_STREAM("Contacts generated for robot "<<contGen.getRobotName()); // <<": "<<contacts);

    urdf2graspit::FileIO fileIO(outputDir, contGen.getOutStructure());
    if (!fileIO.initOutputDir(contGen.getRobotName()))
    {
        ROS_ERROR_STREAM("Could not initialize output directory "
            <<outputDir<<" for robot "<<contGen.getRobotName());
        return 0;
    }
    if (!fileIO.writeContacts(contGen.getRobotName(), contacts))
    {
        ROS_ERROR("Could not write files");
        return 0;
    }
    
    ROS_INFO("Cleaning up...");
    bool deleteOutputRedirect = true;
    converter.cleanup(deleteOutputRedirect);
    
    ROS_INFO("Done.");
    return 0;
}
