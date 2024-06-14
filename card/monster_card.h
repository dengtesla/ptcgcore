#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"
#include "common/error_code.h"

#include "card.pb.h"

namespace ptcgcore {

class MonsterCard : public ICard {
 public:
  MonsterCard(const std::string& name) : ICard(name) {
    card_type_ = card::common::BasicCardType::MONSTER;
  }
  MonsterCard(const card::Card& card) : ICard(card) {
    const auto& monster_msg = card.monster_card();
    elemental_type_ = monster_msg.elemental_type();
    weakness_ = monster_msg.weakness();
    resistance_ = monster_msg.resistance();
    monster_phase_ = monster_msg.monster_phase();
    for (const auto& attack : monster_msg.attack()) {
      attacks_.push_back(attack);
    }
    ability_.CopyFrom(monster_msg.ability());
    hp_ = monster_msg.hp();
  }

  int GetAttack(const int& attack_num, card::Attack& attack) const {
    if (attack_num >= attacks_.size()) return OUT_OF_RANGE_ERROR;
    attack = attacks_.at(attack_num);
    return SUCC;
  }

  int HP() const { return hp_; }

  card::common::ElementalType GetWeakness() const { return weakness_; }
  card::common::ElementalType GetElementalType() const { return elemental_type_; }
  card::common::ElementalType GetResistance() const { return resistance_; }

 private:
  int hp_;
  card::common::ElementalType elemental_type_;
  card::common::ElementalType weakness_;
  card::common::ElementalType resistance_;
  card::common::MonsterPhase monster_phase_;
  std::vector<card::Attack> attacks_;
  card::Ability ability_;

  // pose 信息
  
};

}