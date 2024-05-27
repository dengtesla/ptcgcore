#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "spdlog/spdlog.h"

#include "world.h"

#include "command.pb.h"
#include "card.pb.h"

using std::placeholders::_1;
using CmdPtr = std::shared_ptr<playground::Command>;
using cardptr = std::shared_ptr<ptcgcore::card::Card>;

class PtcgWorld : public rclcpp::Node {
 public:
  PtcgWorld()
  : Node("ptcg_world") {
    // TODO: add config
    player1_sub_ = this->create_subscription<std_msgs::msg::String>("player1", 10,
        std::bind(&PtcgWorld::PlayerCallback, this, _1));
    
    player2_sub_ = this->create_subscription<std_msgs::msg::String>("player2", 10,
        std::bind(&PtcgWorld::PlayerCallback, this, _1));

    world_ptr_ = std::make_shared<ptcgcore::World>(
        "/mnt/public/szdeng/ptcgcore/config/world_config.pb.txt");
    Setup();
  }
 private:
  ptcgcore::WorldPtr world_ptr_ = nullptr;
  // 接受 player1 的指令
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr player1_sub_;
  // 接受 player2 的指令
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr player2_sub_;
  int curr_turn_player_id_ = -1;
  int curr_decision_player_id_ = -1;

  void PlayerCallback(const std_msgs::msg::String::SharedPtr cmd) const;
  // void Player2Callback(const std_msgs::msg::String::SharedPtr cmd) const;
  void Setup();

  void SetMonster();

};
