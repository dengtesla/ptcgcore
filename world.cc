#include "world.h"

namespace ptcgcore {

int World::GetStage(int player_id, StageConstPtr stage) {
  if (player_id == stage_first_->GetPlayerId()) {
    stage = stage_first_;
  } else if (player_id == stage_second_->GetPlayerId()) {
    stage = stage_second_;
  } else {
    // return FAIL;
    return 1;
  }
  // return SUCC;
  return 0;
}


}
