#include "world.h"
#include "place/card_place.h"

#include "spdlog/spdlog.h"

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

World::World(const std::string& world_config_path, const int go_first_player_id) {
  spdlog::info("load world by config {}", world_config_path);
  file::GetProto(world_config_path, world_config_);

  spdlog::info(world_config_.DebugString());

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

}  // namespace ptcgcore
