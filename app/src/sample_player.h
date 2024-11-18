#include <iostream>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "spdlog/spdlog.h"
#include "ptcg_world/srv/command.hpp"
#include "ptcg_world/srv/world_state.hpp"

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
    : Node("sample_player_" + std::to_string(player_id)),
    player_name_(player_name),
    player_id_(player_id) {
    callback_group_ =
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // init publisher
    cmd_publisher_ = this->create_publisher<ProtoMsg>(player_name, 10);
    record_publisher_ = this->create_publisher<ProtoMsg>(player_name + "/cmd_record", 10);

    // init subscriber
    world_cmd_sub_ = this->create_subscription<ProtoMsg>(
        player_name_ + "/command", 100, std::bind(&SamplePlayer::CmdCallback, this, _1), sub_opt);

    get_state_client_ = this->create_client<ptcg_world::srv::WorldState>(
      "ptcg_world/state_request", rmw_qos_profile_services_default, callback_group_);
    set_energy_client_ = this->create_client<ptcg_world::srv::Command>(
      "ptcg_world/set_energy", rmw_qos_profile_services_default, callback_group_);
    take_prize_client_ = this->create_client<ptcg_world::srv::Command>(
      "ptcg_world/take_prize", rmw_qos_profile_services_default, callback_group_);
    set_pokemon_client_ = this->create_client<ptcg_world::srv::Command>(
      "ptcg_world/set_pokemon", rmw_qos_profile_services_default, callback_group_);
    attack_client_ = this->create_client<ptcg_world::srv::Command>(
      "ptcg_world/attack", rmw_qos_profile_services_default, callback_group_);

    // auto callback_group_timer =
    // this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    // timer_ = this->create_wall_timer(500ms, std::bind(&SamplePlayer::timer_callback, this),
    // callback_group_);
  }

 private:
   //  void timer_callback() {
   //    RequestState();
   //  }

  void CmdCallback(const ProtoMsg::SharedPtr cmd);
  // void StateCallback(const ProtoMsg::SharedPtr state_msg);
  void WorldStateCallback(
    rclcpp::Client<ptcg_world::srv::WorldState>::SharedFuture response_future);
  void SimpleCmdCallback(rclcpp::Client<ptcg_world::srv::Command>::SharedFuture response_future);
  int RequestState();

  void GetEnergyCards(const ptcgcore::card::state::StageState& state,
                      std::vector<std::string>& energy_list);

  void SendRequestHooked(const playground::Command& cmd_output) {
    auto request = std::make_shared<ptcg_world::srv::Command::Request>();
    SendMsg(record_publisher_, cmd_output);
    PackMsg(cmd_output, request->cmd);
    rclcpp::Client<ptcg_world::srv::Command>::SharedFuture result;
    if (cmd_output.type() == playground::Command::SET_POKEMON ||
      cmd_output.type() == playground::Command::MOVE_TO_ACTIVE) {
      result = set_pokemon_client_->async_send_request(
        request, std::bind(&SamplePlayer::SimpleCmdCallback, this, std::placeholders::_1));
      spdlog::info("wait for set result...");
    } else if (cmd_output.type() == playground::Command::SET_ENERGY) {
      result = set_energy_client_->async_send_request(
        request, std::bind(&SamplePlayer::SimpleCmdCallback, this, std::placeholders::_1));
    } else if (cmd_output.type() == playground::Command::GET_PRIZE) {
      result = take_prize_client_->async_send_request(
        request, std::bind(&SamplePlayer::SimpleCmdCallback, this, std::placeholders::_1));
    } else if (cmd_output.type() == playground::Command::ATTACK) {
      result = attack_client_->async_send_request(
        request, std::bind(&SamplePlayer::SimpleCmdCallback, this, std::placeholders::_1));
    }
    std::future_status status = result.wait_for(100ms);
  }

  std::string player_name_;
  int player_id_;
  bool receive_state_ = false;
  ptcgcore::card::state::WorldState curr_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;

  // publisher
  rclcpp::Publisher<ProtoMsg>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<ProtoMsg>::SharedPtr record_publisher_;
  rclcpp::Publisher<ProtoMsg>::SharedPtr state_request_pub_;

  rclcpp::Client<ptcg_world::srv::WorldState>::SharedPtr get_state_client_;
  rclcpp::Client<ptcg_world::srv::Command>::SharedPtr set_energy_client_;
  rclcpp::Client<ptcg_world::srv::Command>::SharedPtr take_prize_client_;
  rclcpp::Client<ptcg_world::srv::Command>::SharedPtr set_pokemon_client_;
  rclcpp::Client<ptcg_world::srv::Command>::SharedPtr attack_client_;

  // subscriber
  // 订阅来自 world 的 cmd
  rclcpp::Subscription<ProtoMsg>::SharedPtr world_cmd_sub_;
  // 订阅来自 world 的 state
  rclcpp::Subscription<ProtoMsg>::SharedPtr state_sub_;
};