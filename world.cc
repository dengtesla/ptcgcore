#include "world.h"
#include "place/card_place.h"

namespace ptcgcore {

int World::GetStage(int player_id, StagePtr stage) {
  if (player_id == player1_stage_->GetPlayerId()) {
    stage = player1_stage_;
  } else if (player_id == player2_stage_->GetPlayerId()) {
    stage = player2_stage_;
  } else {
    // return FAIL;
    return 1;
  }
  // return SUCC;
  return 0;
}

// int World::UseFunc(const Func& func) {
//   // 判断 condition 是否满足
//   // 这里不应该不满足
//   for (const auto& cost : func.costs) {
//     // 支付 cost
//   }
//   // 效果处理
//   // stage_first_->
// }


int World::TickSingleAction() {
}

}
