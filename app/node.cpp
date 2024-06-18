#include "ptcg_world.h"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto world = std::make_shared<PtcgWorld>();
  // world->Setup();
  rclcpp::executors::MultiThreadedExecutor exector;
  exector.add_node(world);
  exector.spin();
  // rclcpp::spin(world);
  // rclcpp::shutdown();
  return 0;
}