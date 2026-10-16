// File decoder only: no ROS node, services, publishers or changes to map_server.
#include <cstdio>
#include <nav2_map_server/map_io.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
int main(int argc,char ** argv) {
  if (argc!=3) return 2;
  nav_msgs::msg::OccupancyGrid map;
  if (nav2_map_server::loadMapFromYaml(argv[1],map)!=nav2_map_server::LOAD_MAP_SUCCESS) return 3;
  map.header.frame_id="map";
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> codec;
  codec.serialize_message(&map,&serialized);
  auto raw=serialized.get_rcl_serialized_message();
  FILE * output=std::fopen(argv[2],"wbx");
  if (!output) return 4;
  bool ok=std::fwrite(raw.buffer,1,raw.buffer_length,output)==raw.buffer_length;
  if (std::fclose(output)!=0) ok=false;
  return ok?0:5;
}
