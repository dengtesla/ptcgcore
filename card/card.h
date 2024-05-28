#pragma once

#include <memory>
#include <string>
#include <vector>

#include "buff/buff.h"

#include "card.pb.h"
#include "card/common.pb.h"

namespace ptcgcore {

class World;

// enum CardType {
//   UNKNOWN = 0,
//   MONSTER = 1,
//   TRAINER = 2,
//   ENERGY = 3
// };

// enum SpecialRule {
//   TAG_TEAM = 1,
//   PRISM_STAR = 2,
//   GX = 3,
//   V = 4,
//   V_MAX = 5,
//   V_UNION = 6,
//   V_STAR = 7,
//   RADIANT = 8 // 光辉
// };

// enum ElementalType {
//   GRASS = 1,
//   FIRE = 2,
//   WATER = 3,
//   LIGHTNING = 4,
//   FIGHTING = 5,
//   PSYCHIC = 6,
//   DARKNESS = 7,
//   METAL = 8,
//   DRAGON = 9,
//   COLORLESS = 10,
//   FAIRY = 11,
//   NONE = 12
// };

class ICard {
 public:
  ICard(const std::string &name) : name_(name) {}
  ICard(const card::Card& card) {
    name_ = card.name();
    for (const auto& rule : card.rule()) {
      rule_.emplace_back(
        card::common::CardRule(rule));
    }
    for (const auto& key_word : card.key_word()) {
      key_word_.emplace_back(
        card::common::KeyWord(key_word));
    }
    card_type_ = card.type();
  }
  ~ICard() = default;
  std::string GetName() { return name_; };
  void SetUniqID(const std::string& uniq_id) { uniq_id_ = uniq_id; };
  std::string GetUniqID() { return uniq_id_; };
  // virtual int GetUsableFunc(std::vector<void (*)()> functions) = 0;
 protected:
  std::string name_ = "";
  std::vector<card::common::CardRule> rule_;
  card::common::BasicCardType card_type_ =
      card::common::BasicCardType::UNKNOWN_CARD_TYPE;
  std::vector<card::common::KeyWord> key_word_;
  // 用于标识卡组中的这张卡的 id。
  // 在一副卡组中，每张卡对应不同的 uniq_id_。
  std::string uniq_id_ = "";
};

using CardPtr = std::shared_ptr<ICard>;


} // ptcgcore
