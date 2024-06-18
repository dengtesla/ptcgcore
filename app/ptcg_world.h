#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "spdlog/spdlog.h"

#include "world.h"
#include "pb_transport_helper.h"

#include "command.pb.h"
#include "card.pb.h"
#include "card/card_state.pb.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using CmdPtr = std::shared_ptr<playground::Command>;
using cardptr = std::shared_ptr<ptcgcore::card::Card>;

class PtcgWorld : public rclcpp::Node {
 public:
  PtcgWorld()
  : Node("ptcg_world") {
    // TODO: add config
    callback_group_ =
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // init subscriber
    player1_sub_ = this->create_subscription<ProtoMsg>(
      "player1", 10, std::bind(&PtcgWorld::PlayerCallback, this, _1), sub_opt);
    player2_sub_ = this->create_subscription<ProtoMsg>(
      "player2", 10, std::bind(&PtcgWorld::PlayerCallback, this, _1), sub_opt);

    state_sub_ = this->create_subscription<ProtoMsg>(
      "state_request", 10, std::bind(&PtcgWorld::StateCallback, this, _1), sub_opt);

    // init publisher
    player1_state_pub_ = this->create_publisher<ProtoMsg>("player1/state", 100);
    player2_state_pub_ = this->create_publisher<ProtoMsg>("player2/state", 100);
    player1_cmd_pub_ = this->create_publisher<ProtoMsg>("player1/command", 10);
    player2_cmd_pub_ = this->create_publisher<ProtoMsg>("player2/command", 10);

    // init world
    world_ptr_ = std::make_shared<ptcgcore::World>(
        "/mnt/public/szdeng/ptcgcore/config/world_config.pb.txt");
    player1_id_ = world_ptr_->FirstPlayerID();
    player2_id_ = world_ptr_->SecondPlayerID();

    // setup command(TODO: hardcode, fix it.)
    timer_ = this->create_wall_timer(500ms, std::bind(&PtcgWorld::TimeCallback, this));
  }

  void Setup();

 private:
  ptcgcore::WorldPtr world_ptr_ = nullptr;
  // 接受 player1 的指令
  rclcpp::Subscription<ProtoMsg>::SharedPtr player1_sub_;
  // 接受 player2 的指令
  rclcpp::Subscription<ProtoMsg>::SharedPtr player2_sub_;

  // 监听 player 的 state 请求
  rclcpp::Subscription<ProtoMsg>::SharedPtr state_sub_;
  // 发送给 player1 的 state
  rclcpp::Publisher<ProtoMsg>::SharedPtr player1_state_pub_;
  // 发送给 player2 的 state
  rclcpp::Publisher<ProtoMsg>::SharedPtr player2_state_pub_;

  // 发送给 player1 的命令
  rclcpp::Publisher<ProtoMsg>::SharedPtr player1_cmd_pub_;
  // 发送给 player2 的命令
  rclcpp::Publisher<ProtoMsg>::SharedPtr player2_cmd_pub_;
  int curr_turn_player_id_ = -1;
  int curr_decision_player_id_ = -1;
  int player1_id_ = 1;
  int player2_id_ = 2;
  // 初始化完成后，game_start 为 true
  bool game_start = false;
  // 双方抽取手牌并成功完成战斗宝可梦设置后，setup_finish 为 true
  bool player1_setup_finish = false;
  bool player2_setup_finish = false;
  int player1_set_fail_time = 0;
  int player2_set_fail_time = 0;
  bool setup_finish = false;
  bool first_setup_ = true;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;

  void PlayerCallback(const ProtoMsg::SharedPtr cmd);

  void StateCallback(const ProtoMsg::SharedPtr cmd);

  void TimeCallback() {
    if (first_setup_) {
      first_setup_ = false;
      Setup();
    }
  };

  void Pend();

  void SetMonster(const playground::Command& cmd);
  void SetEnergy(const playground::Command& cmd);
  void Attack(const playground::Command& cmd);

  template <typename T>
  void SendCmdMsg_(const int& player_id, const T& msg_proto) {
    auto publisher = player1_cmd_pub_;
    if (player_id == player2_id_) {
      publisher = player2_cmd_pub_;
    }
    SendMsg(publisher, msg_proto);
  }
};
