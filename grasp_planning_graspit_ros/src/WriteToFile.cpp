#include <grasp_planning_graspit_ros/WriteToFile.h>
#include <sstream>

///////////////////////////////////////////////////////////////////////////////    
bool grasp_planning_graspit_ros::makeDirectoryIfNeeded(const std::string& dPath)
{
    try
    {
        boost::filesystem::path dir(dPath);
        boost::filesystem::path buildPath;

        for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end());
             it != it_end; ++it)
        {
            buildPath /= *it;
            // std::cout << buildPath << std::endl;

            if (!boost::filesystem::exists(buildPath) &&
                    !boost::filesystem::create_directory(buildPath))
            {
                PRINTERROR("Could not create directory " << buildPath);
                return false;
            }
        }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        PRINTERROR(ex.what());
        return false;
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////    
/*bool grasp_planning_graspit_ros::saveToFile(const moveit_msgs::Grasp& msg,
  const std::string& filename, bool asBinary)
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
}*/

///////////////////////////////////////////////////////////////////////////////
bool grasp_planning_graspit_ros::writeGraspMessage(
  const moveit_msgs::Grasp &grasp,
  const std::string &outputDirectory,
  const std::string &filenamePrefix,
  const std::string &ext)
{
  std::stringstream filename;
  filename << outputDirectory << "/" << filenamePrefix << "." << ext;
  std::stringstream filename_txt;
  filename_txt << outputDirectory << "/" << filenamePrefix << "_string." << ext;
  if (!saveToFile(grasp, filename.str(), true))
  {
      PRINTERROR("Could not save grasp to file " << filename.str());
      return false;
  }
  if (!saveToFile(grasp, filename_txt.str(), false))
  {
      PRINTERROR("Could not save grasp to text file " << filename_txt.str());
      return false;
  }
  return true;
}

