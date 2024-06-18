#pragma once

#include <memory>
#include <vector>
#include <set>
#include <deque>

#include "place/card_place.h"
#include "buff/buff.h"
#include "cost/cost.h"

#include "world_config.pb.h"
#include "card/card_state.pb.h"

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
  int GetHand(std::vector<CardPtr>& hand) const;
  int GetLostZone(std::vector<CardPtr>& lost_zone) const;
  int GetDeck(std::vector<CardPtr>& deck, bool hidden = true) const;
  int GetPrizeCard(std::vector<CardPtr>& prize_card, bool hidden = true) const;
  int GetActive(const MonsterPile* active) const;
  int GetPokemonNum() const { return static_cast<int>(monster_pose_.size()); };

  int GetState(
      card::state::StageState& stage_state,
      const bool& hidden_hand = false,
      const bool& hidden_deck = true,
      const bool& hidden_prize = true) const;

  // 修改场上状态，非 const 方法
  int ShuffleDeck();
  int ShufflePrize();
  int SetOnePrize();
  int SetPrizes(const int& num);
  int DrawOneCard();
  int DrawCards(const int& num);
  int SendHand2Deck();
  int ProcessCost(const Cost);
  int InitDeck();
  int SetPokemon(
      const std::string& pokemon_uniq_id,
      const bool& is_active = false);
  int SetEnergy(const std::string& energy_card_uniq_id, const std::string& target_pkm_uniq_id);
  int CheckStage();

 private:
  int player_id_ = -1;
  config::StageConfig config_;
  std::shared_ptr<Stadium> stadium_pose_ = nullptr; // 竞技场
  std::multiset<MonsterPile> monster_pose_; // 战斗+备战区
  std::deque<CardPtr> deck_; // 卡组
  std::vector<CardPtr> prize_card_; // 奖赏卡
  std::multiset<CardPtr> hand_; // 手牌
  std::multiset<CardPtr> discard_; // 弃牌区
  std::multiset<CardPtr> lost_zone_; // 放逐区

  std::vector<IBuff> buff_; // 对玩家的 buff
 private:
  int MonsterPile2State(
      const MonsterPile& monster_pile,
      card::state::MonsterState& monster_state) const;

  int FindCard(const std::string& card_uniq_id, CardPtr card_target,
               card::common::Place& place) const;

  int FindCardFromHand(const std::string& card_uniq_id, const CardPtr card_target) const;

  int FindMonsterFromStage(const std::string& card_uniq_id, const CardPtr card_target) const;

  int FindMonsterPileByID(const std::string& card_uniq_id, const MonsterPile* monster_pile) const;

  int GetCardPtrByIDFromHand(const std::string& card_uniq_id, CardPtr card_ptr) const;
};

using StagePtr = std::shared_ptr<Stage>;
using StageConstPtr = std::shared_ptr<const Stage>;

}
