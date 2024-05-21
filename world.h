#pragma once

#include <iostream>
#include <memory>

#include "spdlog/spdlog.h"

#include "place/stage.h"
#include "place/card_place.h"
#include "place/stage_watcher.h"
#include "common/file.h"

#include "world_config.pb.h"

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
    // stage_first_->InitDeck();
    // spdlog::info("Welcome to spdlog!");
    // config::WorldConfig world_config;
    // file::GetProto("", world_config);
  }

  World(const std::string& world_config_path, const int go_first_player_id = 1) {
    spdlog::info("load world by config {}", world_config_path);
    file::GetProto(world_config_path, world_config_);

    for (const auto& stage_config : world_config_.stage_config()) {
      if (stage_config.id() == go_first_player_id) {
        spdlog::info("init first player's stage.");
        go_first_player_id_ = stage_config.id();
        player1_stage_ = std::make_shared<Stage>(stage_config);
      } else {
        spdlog::info("init second player's stage.");
        go_second_player_id_ = stage_config.id();
        player2_stage_ = std::make_shared<Stage>(stage_config);
      }
    }
    spdlog::info("world init finish!");
  }
  
  int GetStage(int player_id, StagePtr stage);
  int FirstPlayerID() {return go_first_player_id_;};
  int SecondPlayerID() {return go_second_player_id_;};

  // int UseFunc(const Func& func);
  // 1.使用哪一堆卡
  // 2.使用这堆卡的哪张卡效果
  // 3.使用的条件是什么
  // 4.是否需要将行动权转给对方（例：离洞绳）

  int TickSingleAction();

 private:
  int go_first_player_id_;
  int go_second_player_id_;
  StagePtr player1_stage_;
  StagePtr player2_stage_;
  config::WorldConfig world_config_;
};

using WorldPtr = std::shared_ptr<World>;

}
