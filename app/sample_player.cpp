#include "sample_player.h"
#include <fstream>

void SamplePlayer::CmdCallback(const ProtoMsg::SharedPtr cmd) {
  playground::Command cmd_proto;
  DumpMsg(cmd, cmd_proto);
  if (cmd_proto.type() == playground::Command::GAME_START) {
    RequestState();
    spdlog::info("received game start!");
  }
}

void SamplePlayer::StateCallback(const ProtoMsg::SharedPtr state_msg) {
  DumpMsg(state_msg, curr_state_);
  spdlog::info("received state: {}", GetDebugString(curr_state_));
  receive_state_ = true;
  // ptcgcore::file::save_to_pbtxt(curr_state_, "./output2.pb.txt");
}

int SamplePlayer::RequestState() {
  receive_state_ = false;
  ptcgcore::card::state::StateRequest state_request;
  state_request.set_player_id(player_id_);
  SendMsg(state_request_pub_, state_request);

  while (!receive_state_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return SUCC;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto player = std::make_shared<SamplePlayer>("player1", 1);
  rclcpp::executors::MultiThreadedExecutor exector;
  exector.add_node(player);
  exector.spin();
  // rclcpp::spin(player);
  // rclcpp::shutdown();
  return 0;
}