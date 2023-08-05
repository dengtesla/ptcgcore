#include <memory>

#include "card/card.h"

class Ability;
using AbilityPtr = std::shared_ptr<Ability>;

enum ElementalType {
  GRASS = 1,
  FIRE = 2,
  WATER = 3,
  LIGHTNING = 4,
  FIGHTING = 5,
  PSYCHIC = 6,
  DARKNESS = 7,
  METAL = 8,
  DRAGON = 9,
  COLORLESS = 10,
  FAIRY = 11,
  NONE = 12
};

enum MonsterRule {
  
};

enum EvoType {
  BASIC = 1,
  STAGE_1 = 2,
  STAGE_2 = 3,
  VMAX = 4,
  V_EVO = 5,
  V_UNION = 6,
};

class MonsterCard : public ICard {
 public:
  // TODO
  // 特性
  virtual int UseAbility() = 0;
  // 招式（不止一个）
 private:
  ElementalType elemental_type;
  ElementalType weakness;
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