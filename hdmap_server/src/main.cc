#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

#include "hdamp_server/server.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hdmap::HDMapServer>();
  if (!node->Init()) {
    HDMAP_LOG_ERROR("node init failure");
    return EXIT_FAILURE;
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
