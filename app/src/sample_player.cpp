#include "sample_player.h"
#include <fstream>

void SamplePlayer::CmdCallback(const ProtoMsg::SharedPtr cmd) {
  playground::Command cmd_proto, cmd_output;
  cmd_output.set_player_id(player_id_);
  DumpMsg(cmd, cmd_proto);
  if (cmd_proto.type() == playground::Command::GAME_START) {
    spdlog::info("received game start!");
    if ((cmd_proto.game_start_command().first_player() == player_id_ &&
      cmd_proto.game_start_command().first_player_set_finish()) ||
      (cmd_proto.game_start_command().first_player() != player_id_ &&
        cmd_proto.game_start_command().second_player_set_finish())) {
      spdlog::info("already setup.");
      return;
    }
    RequestState();
    spdlog::info("get state succ!!");
    for (const auto& stage_state : curr_state_.stage_state()) {
      if (stage_state.player_id() != player_id_) continue;
      for (const auto& card : stage_state.hand_state().card()) {
        if (card.card_name() == "泪眼蜥") {
          // set 该宝可梦
          cmd_output.set_type(playground::Command::SET_POKEMON);
          cmd_output.mutable_set_pokemon_command()->set_active_pokemon_id(card.card_uniq_id());
          // SendMsg(cmd_publisher_, cmd_output);
          SendRequestHooked(cmd_output);
          return;
        }
      }
      // 没找到泪眼蜥
      cmd_output.set_type(playground::Command::STATE_COMMAND);
      cmd_output.mutable_state_command()->set_state(playground::StateCommand::NO_POKEMON_IN_HAND);
      SendMsg(cmd_publisher_, cmd_output);
      return;
    }
  } else if (cmd_proto.type() == playground::Command::FREE_ACTION) {
    spdlog::info("received free action!");
    RequestState();
    ptcgcore::card::state::StageState my_state;
    for (const auto& stage_state : curr_state_.stage_state()) {
      if (stage_state.player_id() != player_id_) continue;
      my_state = stage_state;
      break;
    }
    // 0. 尽可能铺下手里的泪眼蜥
    int benched_num = my_state.benched_state().size();
    int left_size = 5 - benched_num;
    bool found_card = false;
    for (const auto& card : my_state.hand_state().card()) {
      if (left_size <= 0) break;
      if (card.card_name() != "泪眼蜥") {
        continue;
      }
      left_size--;
      cmd_output.mutable_set_pokemon_command()->add_pokemon_id(card.card_uniq_id());
      found_card = true;
    }
    if (found_card) {
      cmd_output.set_type(playground::Command::SET_POKEMON);
      // SendMsg(cmd_publisher_, cmd_output);
      SendRequestHooked(cmd_output);
      return;
    }
    // 1. 如果手里有能量，且还未填过能量
    if (!my_state.energy_seted()) {
      std::vector<std::string> energy_list;
      GetEnergyCards(my_state, energy_list);
      if (!energy_list.empty()) {
        cmd_output.set_type(playground::Command::SET_ENERGY);
        cmd_output.mutable_set_energy_command()->set_energy_card_id(energy_list[0]);
        if (my_state.active_state().energy_card().size() < 2) {
          // 1.1 如果前场泪眼蜥能量 < 2，则给他填能
          cmd_output.mutable_set_energy_command()->set_target_pokemon(
            my_state.active_state().main_pokemon().card_uniq_id());
        } else {
          const ptcgcore::card::state::MonsterState* select_bench = nullptr;
          size_t curr_min_energy_num = 100;
          for (const auto& pkm_pile : my_state.benched_state()) {
            if (pkm_pile.energy_card().size() < curr_min_energy_num) {
              select_bench = &pkm_pile;
              curr_min_energy_num = pkm_pile.energy_card().size();
            }
          }
          if (select_bench) {
            // 1.2 否则挑选后场能量最少的泪眼蜥填能
            cmd_output.mutable_set_energy_command()->set_target_pokemon(
              select_bench->main_pokemon().card_uniq_id());
          } else {
            // 1.3 fallback 到给战斗场上的泪眼蜥填能
            cmd_output.mutable_set_energy_command()->set_target_pokemon(
              my_state.active_state().main_pokemon().card_uniq_id());
          }
        }
        // SendMsg(cmd_publisher_, cmd_output);
        SendRequestHooked(cmd_output);
        return;
      }
    }
    // 2. 如果满足条件，发起进攻
    if (my_state.active_state().energy_card().size() >= 2) {
      cmd_output.set_type(playground::Command::ATTACK);
      cmd_output.mutable_attack_command()->set_attack_num(1);
      SendRequestHooked(cmd_output);
      return;
    }
    // 3. 否则结束回合
    cmd_output.set_type(playground::Command::END_TURN);
    SendMsg(cmd_publisher_, cmd_output);
  } else if (cmd_proto.type() == playground::Command::DUEL_PRIZE) {
    spdlog::info("received duel prize!");
    cmd_output.set_type(playground::Command::GET_PRIZE);
    cmd_output.mutable_get_prize_command()->set_get_prize_idx(1);
    // SendMsg(cmd_publisher_, cmd_output);
    SendRequestHooked(cmd_output);
  } else if (cmd_proto.type() == playground::Command::NEED_MOVE_TO_ACTIVE) {
    spdlog::info("received need move to active!");
    cmd_output.set_type(playground::Command::MOVE_TO_ACTIVE);
    for (const auto& stage_state : curr_state_.stage_state()) {
      if (stage_state.player_id() != player_id_) continue;
      cmd_output.mutable_move_to_active_command()->set_pkm_id(
        stage_state.benched_state(0).main_pokemon().card_uniq_id());
      break;
    }
    // SendMsg(cmd_publisher_, cmd_output);
    SendRequestHooked(cmd_output);
  }
}

void SamplePlayer::WorldStateCallback(
  rclcpp::Client<ptcg_world::srv::WorldState>::SharedFuture response_future) {
  auto response = response_future.get();
  DumpMsg(response->state, curr_state_);
  spdlog::info("received state: {}", GetDebugString(curr_state_));
  receive_state_ = true;
}

void SamplePlayer::SimpleCmdCallback(
  rclcpp::Client<ptcg_world::srv::Command>::SharedFuture response_future) {
  // pass
}

int SamplePlayer::RequestState() {
  receive_state_ = false;
  ptcgcore::card::state::StateRequest state_request;
  state_request.set_player_id(player_id_);
  while (!get_state_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto request = std::make_shared<ptcg_world::srv::WorldState::Request>();
  PackMsg(state_request, request->cmd);
  auto result = get_state_client_->async_send_request(
    request, std::bind(&SamplePlayer::WorldStateCallback, this, std::placeholders::_1));
  // std::future_status status = result.wait_for(500ms);
  spdlog::info("wait for state...");
  result.wait_for(500ms);
  while (receive_state_ != true) {
  }

  return SUCC;
}

void SamplePlayer::GetEnergyCards(const ptcgcore::card::state::StageState& state,
  std::vector<std::string>& energy_list) {
  for (const auto& card : state.hand_state().card()) {
    if (card.card_name() == "基本水能量") {
      energy_list.emplace_back(card.card_uniq_id());
    }
  }
  return;
}

// sample: ./sample_player player1 1
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::string player_name = argv[1];
  int player_id = atoi(argv[2]);
  spdlog::info("player name: {}, id: {}, let's start!", player_name, player_id);
  auto player = std::make_shared<SamplePlayer>(player_name, player_id);
  rclcpp::executors::MultiThreadedExecutor exector;
  exector.add_node(player);
  exector.spin();
  // rclcpp::spin(player);
  // rclcpp::shutdown();
  return 0;
}