#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "command.pb.h"

using namespace std::chrono_literals;

class SamplePlayer : public rclcpp::Node {
 public:
  SamplePlayer(const std::string& player_name, const int& player_id)
      : Node("sample_player"), player_name_(player_name), player_id_(player_id) {
    publisher_ = this->create_publisher<std_msgs::msg::String>(player_name, 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&SamplePlayer::timer_callback, this));
  }

 private:
  void timer_callback() {
    // add some strategy
    auto msg = std_msgs::msg::String();
    auto command = playground::Command();
    command.set_player_id(player_id_);
    command.set_type(playground::Command::ATTACK);
    command.SerializeToString(&msg.data);
    publisher_->publish(msg);
  }

  std::string player_name_;
  int player_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // 订阅来自 world 的 cmd
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr world_cmd_sub_;
};