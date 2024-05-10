#pragma once

#include <iostream>
#include <memory>

#include "place/stage.h"
#include "place/card_place.h"
#include "place/stage_watcher.h"

namespace ptcgcore {

class Func;

class World {
 public:
  World(int first_player_id = 1, int second_player_id = 2) {
    go_first_player_id_ = first_player_id;
    go_second_player_id_ = second_player_id;
    stage_first_ = std::make_shared<Stage>(first_player_id);
    stage_second_ = std::make_shared<Stage>(second_player_id);
    StageWatcher::getInstance(stage_first_, stage_second_);
    stage_first_->InitDeck();
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
  StagePtr stage_first_;
  StagePtr stage_second_;
};

using WorldPtr = std::shared_ptr<World>;

}
