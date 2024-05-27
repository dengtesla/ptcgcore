#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"

#include "card.pb.h"

namespace ptcgcore {

class Ability;
using AbilityPtr = std::shared_ptr<Ability>;

enum MonsterRule {
  
};

// enum EvoType {
//   BASIC = 1,
//   STAGE_1 = 2,
//   STAGE_2 = 3,
//   VMAX = 4,
//   V_EVO = 5,
//   V_UNION = 6,
// };

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

// 特性
class Ability {
  enum Type {
    CONTINUOUS = 1,
    PASSIVE = 2,
    ACTIVE = 3
  };

 private:
  Type AbilityType;

};

}