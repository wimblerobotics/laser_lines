#include <rclcpp/rclcpp.hpp>

#include "laser_lines/line_extraction_ros.h"

using namespace laser_lines;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("bt");
  auto bt = std::make_shared<LineExtractionROS>(g_node);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {
    bt->run();
    rclcpp::spin_some(g_node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}