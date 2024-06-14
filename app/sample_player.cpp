#include "sample_player.h"


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SamplePlayer>("player1", 1));
  rclcpp::shutdown();
  return 0;
}