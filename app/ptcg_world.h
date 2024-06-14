#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "spdlog/spdlog.h"

#include "world.h"

#include "command.pb.h"
#include "card.pb.h"
#include "card/card_state.pb.h"

using std::placeholders::_1;
using CmdPtr = std::shared_ptr<playground::Command>;
using cardptr = std::shared_ptr<ptcgcore::card::Card>;

class PtcgWorld : public rclcpp::Node {
 public:
  PtcgWorld()
  : Node("ptcg_world") {
    // TODO: add config
    player1_sub_ = this->create_subscription<std_msgs::msg::String>(
        "player1", 10, std::bind(&PtcgWorld::PlayerCallback, this, _1));
    player2_sub_ = this->create_subscription<std_msgs::msg::String>("player2", 10,
        std::bind(&PtcgWorld::PlayerCallback, this, _1));

    player1_state_pub_ = this->create_publisher<std_msgs::msg::String>("player1/state", 10);
    player2_state_pub_ = this->create_publisher<std_msgs::msg::String>("player2/state", 10);

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "state_request", 10, std::bind(&PtcgWorld::StateCallback, this, _1));

    world_ptr_ = std::make_shared<ptcgcore::World>(
        "/mnt/public/szdeng/ptcgcore/config/world_config.pb.txt");
    player1_id_ = world_ptr_->FirstPlayerID();
    player2_id_ = world_ptr_->SecondPlayerID();
    Setup();
  }
 private:
  ptcgcore::WorldPtr world_ptr_ = nullptr;
  // 接受 player1 的指令
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr player1_sub_;
  // 接受 player2 的指令
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr player2_sub_;

  // 监听 player 的 state 请求
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  // 发送给 player1 的 state
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player1_state_pub_;
  // 发送给 player2 的 state
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player2_state_pub_;

  // 发送给 player1 的命令
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player1_cmd_pub_;
  // 发送给 player2 的命令
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player2_cmd_pub_;
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

  void PlayerCallback(const std_msgs::msg::String::SharedPtr cmd);

  void StateCallback(const std_msgs::msg::String::SharedPtr cmd);

  void Setup();
  void Pend();

  void SetMonster(const playground::Command& cmd);
  void SetEnergy(const playground::Command& cmd);
  void Attack(const playground::Command& cmd);

  template <typename T>
  void SendMsg_(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
                const T& msg_proto) {
    auto msg = std_msgs::msg::String();
    msg_proto.SerializeToString(&msg.data);
    publisher->publish(msg);
  }

  template <typename T>
  void SendCmdMsg_(const int& player_id, const T& msg_proto) {
    auto publisher = player1_cmd_pub_;
    if (player_id == player2_id_) {
      publisher = player2_cmd_pub_;
    }
    auto msg = std_msgs::msg::String();
    msg_proto.SerializeToString(&msg.data);
    publisher->publish(msg);
  }
};
