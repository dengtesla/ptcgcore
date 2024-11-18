#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "spdlog/spdlog.h"
#include "ptcg_world/srv/command.hpp"
#include "ptcg_world/srv/world_state.hpp"

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
  PtcgWorld() : Node("ptcg_world") {
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

    // init publisher
    player1_state_pub_ = this->create_publisher<ProtoMsg>("player1/state", 100);
    player2_state_pub_ = this->create_publisher<ProtoMsg>("player2/state", 100);
    player1_cmd_pub_ = this->create_publisher<ProtoMsg>("player1/command", 10);
    player2_cmd_pub_ = this->create_publisher<ProtoMsg>("player2/command", 10);

    // init world
    world_ptr_ =
        std::make_shared<ptcgcore::World>("/mnt/public/szdeng/ptcgcore/config/world_config.pb.txt");
    player1_id_ = world_ptr_->FirstPlayerID();
    player2_id_ = world_ptr_->SecondPlayerID();

    get_state_service_ = this->create_service<ptcg_world::srv::WorldState>(
      "ptcg_world/state_request",
      std::bind(&PtcgWorld::StateCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    set_energy_service_ = this->create_service<ptcg_world::srv::Command>(
      "ptcg_world/set_energy",
      std::bind(&PtcgWorld::SimpleCmdCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    attack_service_ = this->create_service<ptcg_world::srv::Command>(
      "ptcg_world/attack",
      std::bind(&PtcgWorld::AttackCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    take_prize_service_ = this->create_service<ptcg_world::srv::Command>(
      "ptcg_world/take_prize",
      std::bind(&PtcgWorld::SimpleCmdCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    set_pokemon_service_ = this->create_service<ptcg_world::srv::Command>(
      "ptcg_world/set_pokemon",
      std::bind(&PtcgWorld::SimpleCmdCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    // setup command(TODO: hardcode, fix it.)
    timer_ =
      this->create_wall_timer(500ms, std::bind(&PtcgWorld::TimeCallback, this), callback_group_);
  }

  void Setup();

 private:
  ptcgcore::WorldPtr world_ptr_ = nullptr;
  // 接受 player1 的指令
  rclcpp::Subscription<ProtoMsg>::SharedPtr player1_sub_;
  // 接受 player2 的指令
  rclcpp::Subscription<ProtoMsg>::SharedPtr player2_sub_;

  // server 在返回前，如果有其他需要处理的事项（比如：推怪）
  // 那么会先发送 request 给 player，要求 player 进行处理
  // player 处理后才会正常返回。

  // server: 返回现在场上环境
  rclcpp::Service<ptcg_world::srv::WorldState>::SharedPtr get_state_service_;
  // server: 填充能量
  rclcpp::Service<ptcg_world::srv::Command>::SharedPtr set_energy_service_;
  // server: 使用招式攻击
  // 使用招式进行攻击的过程中，可能会需要 player 处理其他事项（比如：拿奖、推怪）
  rclcpp::Service<ptcg_world::srv::Command>::SharedPtr attack_service_;
  // server: 拿去奖赏卡
  rclcpp::Service<ptcg_world::srv::Command>::SharedPtr take_prize_service_;
  // server: 放置宝可梦
  rclcpp::Service<ptcg_world::srv::Command>::SharedPtr set_pokemon_service_;

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
  bool player1_setup_finish_ = false;
  bool player2_setup_finish_ = false;
  int player1_set_fail_time_ = 0;
  int player2_set_fail_time_ = 0;
  bool setup_finish_ = false;
  bool first_setup_ = true;
  bool set_prize_finish_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;

  bool player1_get_prized_ = true;
  bool player2_get_prized_ = true;
  bool player1_moved_pkm_ = true;
  bool player2_moved_pkm_ = true;

  void PlayerCallback(const ProtoMsg::SharedPtr cmd);

  void StateCallback(const ptcg_world::srv::WorldState::Request::SharedPtr request,
    ptcg_world::srv::WorldState::Response::SharedPtr response);

  void SimpleCmdCallback(const ptcg_world::srv::Command::Request::SharedPtr request,
    ptcg_world::srv::Command::Response::SharedPtr response);

  void AttackCallback(const ptcg_world::srv::Command::Request::SharedPtr request,
    ptcg_world::srv::Command::Response::SharedPtr response);

  void TimeCallback() {
    // setup 阶段，只会进入一次。
    if (first_setup_) {
      first_setup_ = false;
      Setup();
    }
  };

  bool PendIsFinished();

  void GameSetupPhase(const playground::Command& cmd_proto);
  void SetMonster(const playground::Command& cmd);
  void SetEnergy(const playground::Command& cmd);
  void Attack(const playground::Command& cmd);
  void GetPrize(const playground::Command& cmd);
  void MoveToActive(const playground::Command& cmd);

  bool PlayerSetFinish(const int& player_id) {
    if (player_id == player1_id_) {
      return player1_setup_finish_;
    } else if (player_id == player2_id_) {
      return player2_setup_finish_;
    }
  }

  template <typename T>
  void SendCmdMsg_(const int& player_id, const T& msg_proto) {
    auto publisher = player1_cmd_pub_;
    if (player_id == player2_id_) {
      publisher = player2_cmd_pub_;
    }
    SendMsg(publisher, msg_proto);
  }
};
