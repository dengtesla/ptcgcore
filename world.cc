#include "world.h"
#include "card_place.h"

namespace ptcgcore {

int World::GetStage(int player_id, StagePtr stage) {
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
