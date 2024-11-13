#include "common/error_code.h"
#include "ptcg_world.h"

#include "command.pb.h"
#include "check_result.pb.h"

void PtcgWorld::StateCallback(const ProtoMsg::SharedPtr cmd_str) {
  ptcgcore::card::state::StateRequest state_request;
  DumpMsg(cmd_str, state_request);
  spdlog::info(state_request.DebugString());

  ptcgcore::card::state::WorldState world_state;
  for (const auto& player_id : world_ptr_->GetPlayersID()) {
    auto* new_stage_state = world_state.add_stage_state();
    ptcgcore::StagePtr stage_ptr = nullptr;
    world_ptr_->GetStage(player_id, stage_ptr);
    if (player_id == state_request.player_id()) {
      stage_ptr->GetState(*new_stage_state, false, true, true);
    } else {
      stage_ptr->GetState(*new_stage_state, true, true, true);
    }
  }

  if (state_request.player_id() == player1_id_) {
    SendMsg(player1_state_pub_, world_state);
  } else {
    SendMsg(player2_state_pub_, world_state);
  }
}

void PtcgWorld::Setup() {
  ptcgcore::StagePtr first_stage = nullptr;
  ptcgcore::StagePtr second_stage = nullptr;
  world_ptr_->GetStage(world_ptr_->FirstPlayerID(), first_stage);
  world_ptr_->GetStage(world_ptr_->SecondPlayerID(), second_stage);
  first_stage->ShuffleDeck();
  second_stage->ShuffleDeck();
  first_stage->DrawCards(7);
  second_stage->DrawCards(7);

  // 发送游戏开始的 cmd
  playground::Command cmd;
  cmd.set_player_id(0);
  cmd.set_type(playground::Command::GAME_START);
  cmd.mutable_game_start_command()->set_first_player(world_ptr_->FirstPlayerID());
  cmd.mutable_game_start_command()->set_first_player_set_finish(false);
  cmd.mutable_game_start_command()->set_second_player_set_finish(false);
  spdlog::info("send player1/2 start cmd!");
  game_start = true;
  SendMsg(player1_cmd_pub_, cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  SendMsg(player2_cmd_pub_, cmd);

  while (!setup_finish_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
  first_stage->SetPrizes(6);
  second_stage->SetPrizes(6);
  curr_turn_player_id_ = world_ptr_->FirstPlayerID();
  curr_decision_player_id_ = world_ptr_->FirstPlayerID();

  set_prize_finish_ = true;
  spdlog::info("setup finished!");
}

void PtcgWorld::SetMonster(const playground::Command& cmd) {
  ptcgcore::StagePtr stage_ptr = nullptr;
  world_ptr_->GetStage(cmd.player_id(), stage_ptr);
  if (stage_ptr->GetPokemonNum() == 0) {
    stage_ptr->SetPokemon(cmd.set_pokemon_command().active_pokemon_id(), true);
  }
  for (const auto& pokemon_id : cmd.set_pokemon_command().pokemon_id()) {
    stage_ptr->SetPokemon(pokemon_id, false);
  }
}

void PtcgWorld::SetEnergy(const playground::Command& cmd) {
  ptcgcore::StagePtr stage_ptr = nullptr;
  world_ptr_->GetStage(cmd.player_id(), stage_ptr);
  stage_ptr->SetEnergy(cmd.set_energy_command().energy_card_id(),
                       cmd.set_energy_command().target_pokemon());
}

void PtcgWorld::Attack(const playground::Command& cmd) {
  world_ptr_->Attack(cmd.player_id(), cmd.attack_command());
  ptcgcore::world::CheckResult player_result;
  world_ptr_->CheckWorld(player_result);
  Pend();
  // 处理 player_result
  playground::Command cmd_out;
  cmd_out.set_player_id(0);
  cmd_out.set_type(playground::Command::DUEL_PRIZE);
  for (const auto& stage_check_result : player_result.stage_check_result()) {
    if (stage_check_result.knock_out_prize_num() > 0) {
      cmd_out.mutable_duel_prize_command()->set_get_prize_num(
          stage_check_result.knock_out_prize_num());
      cmd_out.mutable_duel_prize_command()->set_player_id(stage_check_result.player_id());
      if (stage_check_result.player_id() == player1_id_) {
        player1_get_prized_ = false;
      } else {
        player2_get_prized_ = false;
      }
      SendCmdMsg_(stage_check_result.player_id(), cmd_out);
      if (stage_check_result.player_id() == player1_id_) {
        while (!player1_get_prized_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      } else {
        while (!player2_get_prized_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
    }
  }
  return;
}

void PtcgWorld::GetPrize(const playground::Command& cmd) {
  const auto& player_id = cmd.player_id();
  ptcgcore::StagePtr stage_ptr = nullptr;
  world_ptr_->GetStage(player_id, stage_ptr);
  stage_ptr->GetPrize(cmd.get_prize_command().get_prize_idx());
  if (player_id == player1_id_) {
    player1_get_prized_ = true;
  } else {
    player2_get_prized_ = true;
  }
  return;
}

void PtcgWorld::GameSetupPhase(const playground::Command& cmd_proto) {
  const auto& player_id = cmd_proto.player_id();
  spdlog::error(cmd_proto.DebugString());
  ptcgcore::StagePtr stage_ptr = nullptr;
  world_ptr_->GetStage(player_id, stage_ptr);
  if (cmd_proto.type() == playground::Command::STATE_COMMAND) {
    if (cmd_proto.state_command().state() == playground::StateCommand::NO_POKEMON_IN_HAND) {
      if (player_id == player1_id_) {
        player1_set_fail_time_++;
      } else {
        player2_set_fail_time_++;
      }
      stage_ptr->SendHand2Deck();
      stage_ptr->ShuffleDeck();
      stage_ptr->DrawCards(7);
    } else {
      spdlog::error("unknown state!!!");
      return;
    }
  } else if (cmd_proto.type() == playground::Command::SET_POKEMON) {
    SetMonster(cmd_proto);
    if (player_id == player1_id_) {
      player1_setup_finish_ = true;
    } else {
      player2_setup_finish_ = true;
    }
  }

  // 判定是否都 setup 完成，若都完成，开始补抽牌阶段
  if (player1_setup_finish_ && player2_setup_finish_) {
    if (player1_set_fail_time_ > player2_set_fail_time_) {
      ptcgcore::StagePtr draw_stage_ptr = nullptr;
      world_ptr_->GetStage(player2_id_, draw_stage_ptr);
      draw_stage_ptr->DrawCards(player1_set_fail_time_ - player2_set_fail_time_);
    } else if (player2_set_fail_time_ > player1_set_fail_time_) {
      ptcgcore::StagePtr draw_stage_ptr = nullptr;
      world_ptr_->GetStage(player1_id_, draw_stage_ptr);
      draw_stage_ptr->DrawCards(player2_set_fail_time_ - player1_set_fail_time_);
    }
    setup_finish_ = true;
    // 自由行动
    curr_turn_player_id_ = world_ptr_->FirstPlayerID();
    curr_decision_player_id_ = world_ptr_->FirstPlayerID();
    playground::Command cmd;
    cmd.set_player_id(0);
    cmd.set_type(playground::Command::FREE_ACTION);
    cmd.mutable_free_action_command()->set_player_id(curr_turn_player_id_);
    SendCmdMsg_(curr_turn_player_id_, cmd);
    // SendCmdMsg_(player2_id_, cmd);
    return;
  }

  playground::Command cmd;
  cmd.set_player_id(0);
  cmd.set_type(playground::Command::GAME_START);
  cmd.mutable_game_start_command()->set_first_player(world_ptr_->FirstPlayerID());
  cmd.mutable_game_start_command()->set_first_player_set_finish(
      PlayerSetFinish(world_ptr_->FirstPlayerID()));
  cmd.mutable_game_start_command()->set_second_player_set_finish(
      PlayerSetFinish(world_ptr_->SecondPlayerID()));
  if (!player1_setup_finish_) SendCmdMsg_(player1_id_, cmd);
  if (!player2_setup_finish_) SendCmdMsg_(player2_id_, cmd);
  return;
}

void PtcgWorld::PlayerCallback(const ProtoMsg::SharedPtr cmd_str) {
  playground::Command cmd_proto;
  DumpMsg(cmd_str, cmd_proto);
  spdlog::info("dddddd:" + cmd_proto.DebugString());

  // setup 阶段
  if (game_start && !setup_finish_) {
    GameSetupPhase(cmd_proto);
    return;
  }

  while (!set_prize_finish_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (cmd_proto.type() == playground::Command::SET_ENERGY) {
    SetEnergy(cmd_proto);
  } else if (cmd_proto.type() == playground::Command::SET_POKEMON) {
    SetMonster(cmd_proto);
  } else if (cmd_proto.type() == playground::Command::ATTACK) {
    Attack(cmd_proto);
  } else if (cmd_proto.type() == playground::Command::GET_PRIZE) {
    GetPrize(cmd_proto);
    return;
  }
  // 自由行动
  playground::Command cmd;
  cmd.set_player_id(0);
  cmd.set_type(playground::Command::FREE_ACTION);
  cmd.mutable_free_action_command()->set_player_id(curr_turn_player_id_);
  if (curr_turn_player_id_ == player1_id_) {
    SendCmdMsg_(player1_id_, cmd);
  } else {
    SendCmdMsg_(player2_id_, cmd);
  }
}

void PtcgWorld::Pend() {
  // 1. 检查双方战斗场上是否有宝可梦
  // 若战斗场没有但备战区有，则发送放置宝可梦命令
  // 若战斗场合备战区均没有，则宣判胜利，游戏结束
}