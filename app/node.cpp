#include "ptcg_world.h"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PtcgWorld>());
  rclcpp::shutdown();
  return 0;
}