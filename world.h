#pragma once

#include <iostream>
#include <memory>

#include "stage.h"

namespace ptcgcore {

class World {
 public:
  int GetStage(int player_id, StageConstPtr stage);
  int GAO() {
    std::cout << "1111111" << std::endl;
    return 0;
  }

  int UseCard();

 private:
  int go_first_player_id_;
  StagePtr stage_first_;
  StagePtr stage_second_;
};

using WorldPtr = std::shared_ptr<World>;

}
