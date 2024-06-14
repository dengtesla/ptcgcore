#pragma once

#include <iostream>
#include <memory>

#include "place/stage.h"
#include "place/card_place.h"
#include "place/stage_watcher.h"
#include "common/file.h"

#include "world_config.pb.h"
#include "command.pb.h"

namespace ptcgcore {

class Func;

class World {
 public:
  World(int first_player_id = 1, int second_player_id = 2) {
    go_first_player_id_ = first_player_id;
    go_second_player_id_ = second_player_id;
    player1_stage_ = std::make_shared<Stage>(first_player_id);
    player2_stage_ = std::make_shared<Stage>(second_player_id);
    StageWatcher::getInstance(player1_stage_, player2_stage_);
    players_id_.push_back(first_player_id);
    players_id_.push_back(second_player_id);
  }

  World(const std::string& world_config_path, const int go_first_player_id = 1);

  int GetStage(const int& player_id, StagePtr stage);
  int GetOpponentStage(const int& player_id, StagePtr stage);
  int FirstPlayerID() {return go_first_player_id_;};
  int SecondPlayerID() {return go_second_player_id_;};
  std::vector<int> GetPlayersID() { return players_id_; };

  // int UseFunc(const Func& func);
  // 1.使用哪一堆卡
  // 2.使用这堆卡的哪张卡效果
  // 3.使用的条件是什么
  // 4.是否需要将行动权转给对方（例：离洞绳）

  int TickSingleAction();

  int Attack(const int& player_id, const playground::AttackCommand& command);
  bool IsEnergySatisfy(const std::vector<CardPtr>& energys, const card::Attack& attack);

  int CheckWorld();

 private:
  int go_first_player_id_;
  int go_second_player_id_;
  std::vector<int> players_id_; // 记录当前所有的 player 的 id
  StagePtr player1_stage_;
  StagePtr player2_stage_;
  config::WorldConfig world_config_;
};

using WorldPtr = std::shared_ptr<World>;

}
