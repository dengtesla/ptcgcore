#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"

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
  MonsterCard(std::string& name) {
    Icard(name);
    card_type_ = CardType::MONSTER;
  }
  // TODO
  // 特性
  // virtual int UseAbility(const CostPtr cost) = 0;
  // // 招式（不止一个）
 private:
  ElementalType elemental_type;
  ElementalType weakness;
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