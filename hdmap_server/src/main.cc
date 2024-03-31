#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

int main(int argc, char* argv[]) {
  std::cout << "hdmap server node." << std::endl;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hdmap_server");

  rclcpp::WallRate rate(5);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
