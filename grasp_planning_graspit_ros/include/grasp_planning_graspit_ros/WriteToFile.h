#ifndef GRASP_PLANNING_GRASPIT_ROS_WRITETOFILE_H
#define GRASP_PLANNING_GRASPIT_ROS_WRITETOFILE_H

#include <grasp_planning_graspit_ros/LogBindingROS.h>
#include <moveit_msgs/Grasp.h>
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>

namespace grasp_planning_graspit_ros
{

///////////////////////////////////////////////////////////////////////////////
bool makeDirectoryIfNeeded(const std::string& dPath);

///////////////////////////////////////////////////////////////////////////////
// Creates the directory (if required) and saves the grasp message
// \param mode binary if true, saves the file as binary.
template<typename ROSMessage>
bool saveToFile(const ROSMessage& msg, const std::string& filename,
                bool asBinary)
{
  boost::filesystem::path p(filename);
  boost::filesystem::path dir = p.parent_path();
  if (!makeDirectoryIfNeeded(dir.string()))
  {
    ROS_ERROR_STREAM("Could not create directory " << dir);
    return false;
  }


  std::ios_base::openmode mode;
  if (asBinary) mode = std::ios::out | std::ios::binary;
  else mode = std::ios::out;

  std::ofstream ofs(filename.c_str(), mode);

  if (!ofs.is_open())
  {
      ROS_ERROR("File %s cannot be opened.", filename.c_str());
      return false;
  }

  if (asBinary)
  {
      uint32_t serial_size = ros::serialization::serializationLength(msg);
      boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
      ros::serialization::OStream ostream(obuffer.get(), serial_size);
      ros::serialization::serialize(ostream, msg);
      ofs.write((char*) obuffer.get(), serial_size);
  }
  else
  {
      ofs<<msg;
  }
  ofs.close();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Writes the grasp message once as binary and once as text.
// @param filenamePrefix the prefix to the file. For the binary version,
//  this will be
//  outputDir + '/' + filename + . + \e ext.
//  For the string version, this will be
//  outputDir + '/' + filename + _string + . + \e ext.
bool writeGraspMessage(
  const moveit_msgs::Grasp &grasp,
  const std::string &outputDirectory,
  const std::string &filenamePrefix = "Grasp",
  const std::string &ext= "msg");

}  // namespace

#endif  // GRASP_PLANNING_GRASPIT_ROS_WRITETOFILE_H
