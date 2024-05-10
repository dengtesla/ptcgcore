#pragma once

#include <memory>
#include <string>
#include <vector>

#include "buff/buff.h"

namespace ptcgcore {

class World;

enum CardType {
  UNKNOWN = 0,
  MONSTER = 1,
  TRAINER = 2,
  ENERGY = 3
};

enum SpecialRule {
  TAG_TEAM = 1,
  PRISM_STAR = 2,
  GX = 3,
  V = 4,
  V_MAX = 5,
  V_UNION = 6,
  V_STAR = 7,
  RADIANT = 8 // 光辉
};

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

class ICard {
 public:
  ICard(std::string &name) : name_(name) {}
  ~ICard() = default;
  std::string GetName() { return name_; };
  virtual int GetUsableFunc(std::vector<void (*)()> functions) = 0;
 private:
  std::string name_ = "";
  std::vector<SpecialRule> rule_;
  CardType card_type_ = CardType::UNKNOWN;
};

using CardPtr = std::shared_ptr<ICard>;


} // ptcgcore
