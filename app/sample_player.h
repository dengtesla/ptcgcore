#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "spdlog/spdlog.h"

#include "common/error_code.h"
#include "common/file.h"
#include "pb_transport_helper.h"

#include "command.pb.h"
#include "card/card_state.pb.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SamplePlayer : public rclcpp::Node {
 public:
  SamplePlayer(const std::string& player_name, const int& player_id)
      : Node("sample_player"), player_name_(player_name), player_id_(player_id) {
    callback_group_ =
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // init publisher
    cmd_publisher_ = this->create_publisher<ProtoMsg>(player_name, 10);
    state_request_pub_ = this->create_publisher<ProtoMsg>("state_request", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&SamplePlayer::timer_callback, this));

    // init subscriber
    world_cmd_sub_ = this->create_subscription<ProtoMsg>(
      "player1/command", 100, std::bind(&SamplePlayer::CmdCallback, this, _1), sub_opt);
    state_sub_ = this->create_subscription<ProtoMsg>(
      "player1/state", 100, std::bind(&SamplePlayer::StateCallback, this, _1), sub_opt);
  }

 private:
  void timer_callback() {
    // add some strategy
    // auto msg = std_msgs::msg::String();
    // auto command = playground::Command();
    // command.set_player_id(player_id_);
    // command.set_type(playground::Command::ATTACK);
    // command.SerializeToString(&msg.data);
    // cmd_publisher_->publish(msg);
    // spdlog::info("send msg");
    RequestState();
  }

  void CmdCallback(const ProtoMsg::SharedPtr cmd);
  void StateCallback(const ProtoMsg::SharedPtr state_msg);
  int RequestState();

  std::string player_name_;
  int player_id_;
  bool receive_state_ = false;
  ptcgcore::card::state::WorldState curr_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;

  // publisher
  rclcpp::Publisher<ProtoMsg>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<ProtoMsg>::SharedPtr state_request_pub_;

  // subscriber
  // 订阅来自 world 的 cmd
  rclcpp::Subscription<ProtoMsg>::SharedPtr world_cmd_sub_;
  // 订阅来自 world 的 state
  rclcpp::Subscription<ProtoMsg>::SharedPtr state_sub_;
};