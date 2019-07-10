#include <ros/ros.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/io/message_conversion.hpp>
#include <lanelet2_extension/io/autoware_parser.hpp>

#include <lanelet2_msgs/MapXML.h>
#include <lanelet2_msgs/MapBin.h>

#define MGRS_PROJECTOR "mgrs"
#define UTM_PROJECTOR "utm"

void printUsage()
{
  std::cout << "Usage:" << std::endl
            << "rosrun lanelet_map_loader lanelet_map_loader _map_file:=<path to osm file>" << std::endl;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "lanelet_map_loader");
  ros::NodeHandle rosnode;
  ros::NodeHandle private_rosnode("~");

  std::string map_path = "";
  std::string projector_type = MGRS_PROJECTOR;
  lanelet2_msgs::MapBin map_bin_msg;

  if(!private_rosnode.hasParam("map_file"))
  {
    ROS_FATAL_STREAM("failed find map_file parameter! No file to load");
    printUsage();
    return 1;
  }

  private_rosnode.getParam("map_file", map_path);
  private_rosnode.param<std::string>("projector", projector_type, MGRS_PROJECTOR);

  std::shared_ptr<lanelet::Projector> projector_ptr;
  if(projector_type == MGRS_PROJECTOR)
  {
    projector_ptr = std::make_shared<lanelet::projection::MGRSProjector>();
  }
  else if(projector_type == UTM_PROJECTOR)
  {
    if(!private_rosnode.hasParam("origin_latitude") || !private_rosnode.hasParam("origin_longitude"))
    {
      ROS_FATAL_STREAM("You must specify origin_latitude and origin_longitude parameter in order to use utm projector");
      return 1;
    }
    double origin_latitude = 0;
    double origin_longitude = 0;
    private_rosnode.param<double>("origin_latitude", origin_latitude, origin_latitude);
    private_rosnode.param<double>("origin_longitude", origin_longitude, origin_longitude);
    projector_ptr = std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({origin_latitude, origin_longitude}));
  }
  else
  {
    ROS_ERROR_STREAM( "Projector " << projector_type << "is not supported." << std::endl
                   << "Using MGRS projector instead." << std::endl);
    projector_ptr = std::make_shared<lanelet::projection::MGRSProjector>();
  }

  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr map = load(map_path, *projector_ptr, &errors);

  // publisher to visualise lanelet elements within rviz
  ros::Publisher map_bin_pub = rosnode.advertise<lanelet2_msgs::MapBin>("/lanelet_map", 1, true);

  lanelet_utils::Map::toBinMsg(map, map_bin_msg);
  lanelet::io_handlers::AutowareOsmParser::parseVersions(map_path, map_bin_msg.format_version, map_bin_msg.map_version);
  map_bin_pub.publish(map_bin_msg);

  ros::spin();

  return 0;
}
