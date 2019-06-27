#include <ros/ros.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Parser.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/OsmFile.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <sstream>

#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/io/message_conversion.hpp>

#include <lanelet2_msgs/MapXML.h>
#include <lanelet2_msgs/MapBin.h>

#include <iostream>
#include <fstream>
#include <string>

lanelet::LaneletMapPtr lanelet_map;
lanelet::LaneletMapPtr lanelet_map2;

void xmlMapCallback(lanelet2_msgs::MapXML msg)
{
  int status = lanelet_utils::Map::fromXMLMsg(msg, lanelet_map);
  std::cout << status << std::endl;
  // for (auto lanelet: lanelet_map->laneletLayer)
  // {
  //   std::cout << lanelet << std::endl;
  // }

}

void binMapCallback(lanelet2_msgs::MapBin msg)
{
  lanelet_utils::Map::fromBinMsg(msg, lanelet_map2);
  for (auto lanelet: lanelet_map2->laneletLayer)
  {
    std::cout << lanelet << std::endl;
  }
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int main (int argc, char **argv)
{
  ros::init(argc, argv, "lanelet_map_subscriber");
  ros::NodeHandle rosnode;

  ros::Subscriber xml_map_sub = rosnode.subscribe("/lanelet_map_xml", 10000,  xmlMapCallback);
  ros::Subscriber bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 10000,  binMapCallback);

  ros::spin();

  return 0;
}
