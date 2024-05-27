#pragma once

#include <memory>
#include <vector>

#include "place/card_place.h"
#include "buff/buff.h"
#include "cost/cost.h"

#include "world_config.pb.h"

namespace ptcgcore {

class Stage {
 public:
  Stage(const int& player_id) : player_id_(player_id) {}
  Stage(const config::StageConfig& stage_config) : player_id_(stage_config.id()) {
    config_.CopyFrom(stage_config);
    InitDeck();
  }
  // 获取场上状态，const 方法
  int GetPlayerId() const {return player_id_;};
  int GetBench(std::vector<MonsterPile>& bench) const;
  int GetHand(std::vector<ICardPlace>& hand) const;
  int GetLostZone(std::vector<ICardPlace>& lost_zone) const;
  int GetDeck(std::vector<ICardPlace>& deck, bool hidden = true) const;
  int GetPrizeCard(std::vector<CardPtr>& prize_card, bool hidden = true) const;

  // 修改场上状态，非 const 方法
  int ShuffleDeck();
  int ShufflePrize();
  int DrawOneCard();
  int ProcessCost(const Cost);
  int InitDeck();
 private:
  int player_id_ = -1;
  config::StageConfig config_;
  Stadium stadium_pose; // 竞技场
  MonsterPile active_pose; // 战斗场
  std::vector<MonsterPile> bench_; // 备战区
  std::vector<CardPtr> deck_; // 卡组
  std::vector<CardPtr> prize_card_; // 奖赏卡
  std::vector<CardPtr> hand_; // 手牌
  std::vector<CardPtr> lost_zone_; // 放逐区

  std::vector<IBuff> buff_; // 对玩家的 buff
};

using StagePtr = std::shared_ptr<Stage>;
using StageConstPtr = std::shared_ptr<const Stage>;

}
