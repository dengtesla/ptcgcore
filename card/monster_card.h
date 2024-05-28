#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"

#include "card.pb.h"

namespace ptcgcore {

class MonsterCard : public ICard {
 public:
  MonsterCard(const std::string& name) : ICard(name) {
    card_type_ = card::common::BasicCardType::MONSTER;
  }
  MonsterCard(const card::Card& card) : ICard(card) {
    
  }
  // TODO
  // 特性
  // virtual int UseAbility(const CostPtr cost) = 0;
  // // 招式（不止一个）
 private:
  card::common::ElementalType elemental_type;
  card::common::ElementalType weakness;
  // pose 信息
  
};

}