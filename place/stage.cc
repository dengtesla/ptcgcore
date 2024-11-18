#include <iostream>
#include <memory>
#include <algorithm>
#include <random>

#include "common/error_code.h"
#include "place/stage.h"
#include "common/file.h"
#include "card/monster_card.h"
#include "card/card_factory.h"

#include "world_config.pb.h"
#include "deck_config.pb.h"

namespace ptcgcore {

int Stage::InitDeck() {
  // use stage_config.deck_path to init deck.
  int rtn = SUCC;
  const auto& deck_path = config_.deck_path();
  spdlog::info("deck_path: {}",deck_path);
  config::DeckConfig deck;
  rtn = file::GetProto(deck_path, deck);
  const auto& card_factory = CardFactory::Instance();
  int cnt = 0;
  for (const auto& card_name : deck.main_deck().card()) {
    auto new_card = card_factory->CreateCard(card_name);
    new_card->SetUniqID(std::to_string(player_id_) + "_" + std::to_string(cnt++));
    deck_.push_front(new_card);
    spdlog::debug("add card: {}",card_name);
  }
  return rtn;
}

int Stage::DrawOneCard() {
  if (deck_.empty()) {
    return DECK_EMPTY_ERROR;
  }
  const auto card = deck_.front();
  hand_.insert(card);
  deck_.pop_front();
  return SUCC;
}

int Stage::DrawCards(const int& num) {
  for (int i = 0; i < num; i++) {
    ERR_CHECK(DrawOneCard());
  }
  return SUCC;
}

int Stage::SendHand2Deck() {
  for (const auto& card : hand_) {
    deck_.push_front(card);
  }
  hand_.clear();
  return SUCC;
}

int Stage::SetOnePrize() {
  if (deck_.empty()) {
    return DECK_EMPTY_ERROR;
  }
  const auto card = deck_.front();
  prize_card_.push_back(card);
  deck_.pop_front();
  return SUCC;
}

int Stage::SetPrizes(const int& num) {
  for (int i =0; i < num; i++) {
    ERR_CHECK(SetOnePrize());
  }
  return SUCC;
}

int Stage::ShuffleDeck() {
  std::vector<CardPtr> tmp_deck;
  tmp_deck.reserve(deck_.size());
  while (!deck_.empty()) {
    tmp_deck.emplace_back(deck_.front());
    deck_.pop_front();
  }
  std::random_shuffle(tmp_deck.begin(), tmp_deck.end());
  for (const auto& card : tmp_deck) {
    deck_.push_front(card);
  }
  return SUCC;
}

int Stage::SetPokemon(
    const std::string& pokemon_uniq_id,
    const bool& is_active) {
  CardPtr target_pkm = nullptr;
  for (const auto& card : hand_) {
    if (card->GetUniqID() == pokemon_uniq_id) {
      target_pkm = card;
      break;
    }
  }
  MonsterPile new_pile;
  new_pile.monsters.push_back(target_pkm);
  new_pile.main_monster = target_pkm;
  if (is_active) {
    new_pile.is_active = true;
  }
  monster_pose_.insert(new_pile);
  hand_.erase(target_pkm);
  return SUCC;
}

int Stage::SetActive(
  const std::string& pokemon_uniq_id) {
  CardPtr target_pkm = nullptr;
  for (const auto& pile : monster_pose_) {
    if (pile.main_monster->GetUniqID() == pokemon_uniq_id) {
      auto new_pile = pile;
      new_pile.is_active = true;
      monster_pose_.erase(pile);
      monster_pose_.insert(new_pile);
      return SUCC;
    }
  }
  return CARD_NOT_FOUND_ERROR;
}

int Stage::SetEnergy(const std::string& energy_card_uniq_id,
                     const std::string& target_pkm_uniq_id) {
  int rtn = SUCC;
  // TODO
  {
    std::lock_guard<std::mutex> mylockguard(mylock);
    MonsterPile monster_pile;
    rtn = FindMonsterPileByID(target_pkm_uniq_id, monster_pile);
    ERR_CHECK(rtn);
    MonsterPile new_monster_pile = monster_pile;
    CardPtr energy_card_ptr = nullptr;
    rtn = GetCardPtrByIDFromHand(energy_card_uniq_id, energy_card_ptr);
    ERR_CHECK(rtn);

    new_monster_pile.energys.emplace_back(energy_card_ptr);
    monster_pose_.erase(monster_pile);
    monster_pose_.insert(new_monster_pile);
    hand_.erase(energy_card_ptr);
    energy_seted_ = true;
  }
  return SUCC;
}

int Stage::GetState(
    card::state::StageState& stage_state,
    const bool& hidden_hand,
    const bool& hidden_deck,
    const bool& hidden_prize) const {
  int rtn = SUCC;
  stage_state.set_player_id(player_id_);
  stage_state.set_energy_seted(energy_seted_);
  // active and bench
  for (const auto& pile : monster_pose_) {
    card::state::MonsterState monster_state;
    if (pile.is_active) {
      rtn = MonsterPile2State(pile, *stage_state.mutable_active_state());
      ERR_CHECK(rtn);
    } else {
      auto bench_state = stage_state.add_benched_state();
      rtn = MonsterPile2State(pile, *bench_state);
      ERR_CHECK(rtn);
    }
  }
  // discard
  for (const auto& discard : discard_) {
    auto card_state = stage_state.mutable_discard_state()->add_card();
    card_state->set_card_name(discard->GetName());
    card_state->set_card_uniq_id(discard->GetUniqID());
  }
  // lost zone
  for (const auto& lost_card : lost_zone_) {
    auto card_state = stage_state.mutable_lost_zone_state()->add_card();
    card_state->set_card_name(lost_card->GetName());
    card_state->set_card_uniq_id(lost_card->GetUniqID());
  }
  // stadium_state
  if (stadium_pose_) {
    auto stadium_state = stage_state.mutable_stadium_state()->add_card();
    stadium_state->set_card_name(stadium_pose_->stadium_card->GetName());
    stadium_state->set_card_uniq_id(stadium_pose_->stadium_card->GetUniqID());
  }
  // hidden msg
  if (hidden_hand) {
    stage_state.set_hand_card_num(hand_.size());
  } else {
    for (const auto& card : hand_) {
      auto card_state = stage_state.mutable_hand_state()->add_card();
      card_state->set_card_name(card->GetName());
      card_state->set_card_uniq_id(card->GetUniqID());
    }
  }
  if (hidden_deck) {
    stage_state.set_deck_card_num(deck_.size());
  } else {
    // so ugly
    std::vector<CardPtr> tmp_deck;
    auto deck_backup = deck_;
    tmp_deck.reserve(deck_backup.size());
    while (!deck_backup.empty()) {
      tmp_deck.emplace_back(deck_backup.front());
      auto card_state = stage_state.mutable_deck_state()->add_card();
      card_state->set_card_name(deck_backup.front()->GetName());
      card_state->set_card_uniq_id(deck_backup.front()->GetUniqID());
      deck_backup.pop_front();
    }
  }
  if (hidden_prize) {
    stage_state.set_prize_card_num(prize_card_.size());
  } else {
    for (const auto& card : prize_card_) {
      auto card_state = stage_state.mutable_prize_state()->add_card();
      card_state->set_card_name(card->GetName());
      card_state->set_card_uniq_id(card->GetUniqID());
    }
  }
  return SUCC;
}

int Stage::MonsterPile2State(
    const MonsterPile& monster_pile,
    card::state::MonsterState& monster_state) const {
  // main
  if (monster_pile.main_monster != nullptr) {
    monster_state.mutable_main_pokemon()->set_card_name(
      monster_pile.main_monster->GetName());
    monster_state.mutable_main_pokemon()->set_card_uniq_id(
      monster_pile.main_monster->GetUniqID());
  }
  // monsters
  for (const auto& card : monster_pile.monsters) {
    auto* add_card = monster_state.mutable_pokemon_pile()->Add();
    add_card->set_card_name(card->GetName());
    add_card->set_card_uniq_id(card->GetUniqID());
  }
  // item
  for (const auto& card : monster_pile.items) {
    auto* add_card = monster_state.mutable_item()->Add();
    add_card->set_card_name(card->GetName());
    add_card->set_card_uniq_id(card->GetUniqID());
  }
  // energy
  for (const auto& card : monster_pile.energys) {
    auto* add_card = monster_state.mutable_energy_card()->Add();
    add_card->set_card_name(card->GetName());
    add_card->set_card_uniq_id(card->GetUniqID());
  }
  // damage
  monster_state.set_damage(monster_pile.damage_counters);
  return SUCC;
}

int Stage::FindMonsterPileByID(const std::string& card_uniq_id, MonsterPile& monster_pile) const {
  for (auto pile : monster_pose_) {
    for (const auto& card : pile.monsters) {
      if (card->GetUniqID() == card_uniq_id) {
        monster_pile = pile;
        return SUCC;
      }
    }
    for (const auto& card : pile.energys) {
      if (card->GetUniqID() == card_uniq_id) {
        monster_pile = pile;
        return SUCC;
      }
    }
    for (const auto& card : pile.items) {
      if (card->GetUniqID() == card_uniq_id) {
        monster_pile = pile;
        return SUCC;
      }
    }
  }
  return CARD_NOT_FOUND_ERROR;
}

int Stage::GetCardPtrByIDFromHand(const std::string& card_uniq_id, CardPtr& card_ptr) const {
  for (const auto& card : hand_) {
    if (card->GetUniqID() == card_uniq_id) {
      card_ptr = card;
      return SUCC;
    }
  }
  return CARD_NOT_FOUND_ERROR;
}

int Stage::GetActive(const MonsterPile** active) const {
  *active = nullptr;
  for (const auto& pile : monster_pose_) {
    if (pile.is_active) {
      *active = &pile;
      return SUCC;
    }
  }
  return CARD_NOT_FOUND_ERROR;
}

int Stage::UpdateMonsterPile(const MonsterPile& prev_monster_pile,
  const MonsterPile& new_monster_pile) {
  monster_pose_.erase(prev_monster_pile);
  monster_pose_.insert(new_monster_pile);
  return SUCC;
}

int Stage::CheckStage(world::StageCheckResult& check_result) {
  check_result.set_player_id(player_id_);
  for (auto& pile : monster_pose_) {
    auto monster = std::static_pointer_cast<MonsterCard>(pile.main_monster);
    if (pile.damage_counters >= monster->HP()) {
      // 昏厥
      check_result.set_knock_out_prize_num(1);
      if (pile.is_active) {
        check_result.set_need_move_to_active(true);
      }
      for (auto& card : pile.monsters) {
        discard_.insert(card);
      }
      for (auto& card : pile.energys) {
        discard_.insert(card);
      }
      for (auto& card : pile.items) {
        discard_.insert(card);
      }
      monster_pose_.erase(pile);
    }
  }
  return SUCC;
}

int Stage::GetPrize(const int& idx) {
  const int target_idx = idx % prize_card_.size();
  auto card = prize_card_[target_idx];
  hand_.insert(card);
  prize_card_.erase(prize_card_.begin() + target_idx);

  spdlog::debug("玩家 " + std::to_string(player_id_) + " 拿取 1 张奖赏卡，剩余奖赏卡数：" + std::to_string(prize_card_.size()));
  return SUCC;
}

int Stage::SetEnergySeted(const bool energy_seted) {
  energy_seted_ = energy_seted;
  return SUCC;
}

bool Stage::IsActive(const std::string& pkm_id) const {
  for (auto& pile : monster_pose_) {
    if (pile.main_monster->GetUniqID() == pkm_id) {
      return pile.is_active;
    }
  }
  return false;
}

}  // namespace ptcgcore