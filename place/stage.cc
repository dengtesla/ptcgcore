#include <iostream>
#include <memory>
#include <algorithm>
#include <random>

#include "place/stage.h"
#include "common/file.h"
#include "card/monster_card.h"
#include "card/card_factory.h"

#include "world_config.pb.h"
#include "deck_config.pb.h"

namespace ptcgcore {

int Stage::InitDeck() {
  // use stage_config.deck_path to init deck.
  int rtn = 1;
  const auto& deck_path = config_.deck_path();
  spdlog::info("deck_path: {}",deck_path);
  config::DeckConfig deck;
  rtn = file::GetProto(deck_path, deck);
  const auto& card_factory = CardFactory::Instance();
  int cnt = 0;
  for (const auto& card_name : deck.main_deck().card()) {
    auto new_card = card_factory->CreateCard(card_name);
    new_card->SetUniqID(std::to_string(player_id_) + "_" + std::to_string(cnt));
    deck_.push_front(new_card);
    spdlog::debug("add card: {}",card_name);
  }
  return rtn;
}

int Stage::DrawOneCard() {
  const auto card = deck_.front();
  hand_.insert(card);
  deck_.pop_front();
  return 1;
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
  return 1;
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
  // if (target_pkm == nullptr) {}
  if (is_active) {
    active_pose_.monsters.push_back(target_pkm);
    active_pose_.main_monster = target_pkm;
  } else {
    MonsterPile new_pile;
    new_pile.monsters.push_back(target_pkm);
    new_pile.main_monster = target_pkm;
    bench_.emplace_back(new_pile);
  }
  hand_.erase(target_pkm);
  return true;
}

int Stage::GetState(
    card::state::StageState& stage_state,
    const bool& hidden_hand,
    const bool& hidden_deck,
    const bool& hidden_prize) const {
  stage_state.set_player_id(player_id_);
  // active
  card::state::MonsterState active_state;
  MonsterPile2State(active_pose_, active_state);
  stage_state.mutable_active_state()->CopyFrom(active_state);
  // bench
  for (const auto& bench_pile : bench_) {
    auto bench_state = stage_state.add_benched_state();
    MonsterPile2State(bench_pile, *bench_state);
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
  auto stadium_state = stage_state.mutable_stadium_state()->add_card();
  stadium_state->set_card_name(stadium_pose_.stadium_card->GetName());
  stadium_state->set_card_uniq_id(stadium_pose_.stadium_card->GetUniqID());
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
  return true;
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
  return true;
}

}  // namespace ptcgcore