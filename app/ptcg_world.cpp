#include "common/error_code.h"
#include "ptcg_world.h"

#include "command.pb.h"

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
  SendMsg(player1_cmd_pub_, cmd);
  SendMsg(player2_cmd_pub_, cmd);
  spdlog::info("send player1/2 start cmd!");
  game_start = true;

  while (!setup_finish) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
  spdlog::info("setup finished!");
  first_stage->SetPrizes(6);
  second_stage->SetPrizes(6);
  curr_turn_player_id_ = world_ptr_->FirstPlayerID();
  curr_decision_player_id_ = world_ptr_->FirstPlayerID();
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
  world_ptr_->CheckWorld();
  Pend();
}

void PtcgWorld::PlayerCallback(const ProtoMsg::SharedPtr cmd_str) {
  playground::Command cmd_proto;
  DumpMsg(cmd_str, cmd_proto);
  spdlog::info(cmd_proto.DebugString());

  // setup 阶段
  if (game_start && !setup_finish) {
    const auto player_id = cmd_proto.player_id();
    ptcgcore::StagePtr stage_ptr = nullptr;
    world_ptr_->GetStage(player_id, stage_ptr);
    if (cmd_proto.type() == playground::Command::STATE_COMMAND) {
      if (cmd_proto.state_command().state() == playground::StateCommand::NO_POKEMON_IN_HAND) {
        if (player_id == player1_id_) {
          player1_set_fail_time++;
        } else {
          player2_set_fail_time++;
        }
        stage_ptr->SendHand2Deck();
        stage_ptr->ShuffleDeck();
        stage_ptr->DrawCards(7);
        playground::Command cmd;
        cmd.set_player_id(0);
        cmd.set_type(playground::Command::GAME_START);
        cmd.mutable_game_start_command()->set_first_player(world_ptr_->FirstPlayerID());
        SendCmdMsg_(player_id, cmd);
      }
    } else if (cmd_proto.type() == playground::Command::SET_POKEMON) {
      SetMonster(cmd_proto);
      // 如果已经 setup finish 了，那么这次 set_monster 完成后就可以开始游戏了
      if (player1_setup_finish && player2_setup_finish) {
        setup_finish = true;
        return;
      }
      if (player_id == player1_id_) {
        player1_setup_finish = true;
      } else {
        player2_setup_finish = true;
      }
    }
    if (player1_setup_finish && player2_setup_finish) {
      if (player1_set_fail_time > player2_set_fail_time) {
        ptcgcore::StagePtr draw_stage_ptr = nullptr;
        world_ptr_->GetStage(player2_id_, draw_stage_ptr);
        draw_stage_ptr->DrawCards(player1_set_fail_time - player2_set_fail_time);
        playground::Command cmd;
        cmd.set_player_id(0);
        cmd.set_type(playground::Command::GAME_START);
        cmd.mutable_game_start_command()->set_first_player(world_ptr_->FirstPlayerID());
        SendCmdMsg_(player2_id_, cmd);
      } else if (player2_set_fail_time > player1_set_fail_time) {
        ptcgcore::StagePtr draw_stage_ptr = nullptr;
        world_ptr_->GetStage(player1_id_, draw_stage_ptr);
        draw_stage_ptr->DrawCards(player2_set_fail_time - player1_set_fail_time);
        playground::Command cmd;
        cmd.set_player_id(0);
        cmd.set_type(playground::Command::GAME_START);
        cmd.mutable_game_start_command()->set_first_player(world_ptr_->FirstPlayerID());
        SendCmdMsg_(player1_id_, cmd);
      }
    }
  }

  if (cmd_proto.type() == playground::Command::SET_ENERGY) {
    SetEnergy(cmd_proto);
  } else if (cmd_proto.type() == playground::Command::SET_POKEMON) {
    SetMonster(cmd_proto);
  } else if (cmd_proto.type() == playground::Command::ATTACK) {
    Attack(cmd_proto);
  }
}

void PtcgWorld::Pend() {
  // 1. 检查双方战斗场上是否有宝可梦
  // 若战斗场没有但备战区有，则发送放置宝可梦命令
  // 若战斗场合备战区均没有，则宣判胜利，游戏结束
}