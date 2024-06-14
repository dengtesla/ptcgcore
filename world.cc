#include "world.h"
#include "place/card_place.h"
#include "common/error_code.h"
#include "card/monster_card.h"

#include "spdlog/spdlog.h"

namespace ptcgcore {

int World::GetStage(const int& player_id, StagePtr stage) {
  if (player_id == player1_stage_->GetPlayerId()) {
    stage = player1_stage_;
  } else if (player_id == player2_stage_->GetPlayerId()) {
    stage = player2_stage_;
  } else {
    return PLAYER_NOT_FOUND_ERROR;
  }
  return SUCC;
}

int World::GetOpponentStage(const int& player_id, StagePtr stage) {
  if (players_id_.size() != 2) {
    spdlog::error("player_id is not 2, cannot get opponent!");
    return NOT_IMPLEMENTED_ERROR;
  }
  if (players_id_[0] == player_id) return GetStage(players_id_[1], stage);
  return GetStage(players_id_[0], stage);
}

bool World::IsEnergySatisfy(const std::vector<CardPtr>& energys, const card::Attack& attack) {
  // 做能量匹配
  // TODO
  return true;
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

int World::Attack(const int& player_id, const playground::AttackCommand& command) {
  int rtn = SUCC;
  StagePtr player_stage = nullptr;
  StagePtr oppo_stage = nullptr;
  rtn = GetStage(player_id, player_stage);
  ERR_CHECK(rtn);
  rtn = GetOpponentStage(player_id, oppo_stage);
  ERR_CHECK(rtn);
  MonsterPile* attack_pkm = nullptr;
  rtn = player_stage->GetActive(attack_pkm);
  ERR_CHECK(rtn);
  MonsterPile* oppo_attack_pkm = nullptr;
  rtn = oppo_stage->GetActive(oppo_attack_pkm);
  ERR_CHECK(rtn);
  // 1. 检查是否符合攻击条件
  const auto main_pkm = std::static_pointer_cast<MonsterCard>(attack_pkm->main_monster);
  card::Attack attack;
  ERR_CHECK(main_pkm->GetAttack(command.attack_num(), attack));
  // 1.1 能量满足条件
  if (!IsEnergySatisfy(attack_pkm->energys, attack)) {
    spdlog::error("attack failed, energy not satisfy!");
    return CONDITION_NOT_SATISFY;
  }
  // 1.2 检查是否有 buff 使该宝可梦不能出手（手酸/大手酸/对手招式影响等）
  // TODO
  // 2. 伤害计算
  // 2.1 计算招式描述的伤害
  int damage = attack.damage();
  for (const auto& damage_desc : attack.damage_desc()) {
    if (damage_desc.type() == card::common::DamageWithDesc::THROW_COIN) {
      const int throw_num = damage_desc.throw_coin_desc().num();
      int cnt = 0;
      while (cnt < throw_num) {
        if (rand() % 2 == 1) {
          damage += damage_desc.per_damage();
        }
        cnt++;
      }
    }
  }
  const auto oppo_main_pkm = std::static_pointer_cast<MonsterCard>(oppo_attack_pkm->main_monster);
  // 2.2 计算己方宝可梦附加状态伤害
  // 2.3 计算对方宝可梦附加状态伤害
  // 2.4 计算弱点
  if (oppo_main_pkm->GetWeakness() == main_pkm->GetElementalType()) {
    damage *= 2;
  }
  // 2.5 计算抵抗
  if (oppo_main_pkm->GetResistance() == main_pkm->GetElementalType()) {
    damage -= 30;
  }
  if (damage < 0) damage = 0;
  oppo_attack_pkm->damage_counters += damage;
  return SUCC;
}

int World::CheckWorld() {
  int rtn = SUCC;
  rtn = player1_stage_->CheckStage();
  ERR_CHECK(rtn);
  player2_stage_->CheckStage();
  ERR_CHECK(rtn);
  return SUCC;
}

}  // namespace ptcgcore
