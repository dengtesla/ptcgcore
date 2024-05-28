#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"

#include "card.pb.h"

namespace ptcgcore {

class EnergyCard : public ICard{
 public:
  EnergyCard(const std::string& name) : ICard(name) {
    card_type_ = card::common::BasicCardType::ENERGY;
  }
  EnergyCard(const card::Card& card) : ICard(card) {
    energy_type = card.energy_card().type();
    if (card.type() == card::common::EnergyType::BASIC_ENERGY) {
      elemental_type = card.energy_card().basic_energy_card().element();
    } else if (card.type() == card::common::EnergyType::SPECIAL_ENERGY) {
      elemental_type = card::common::ElementalType::COLORLESS;
    }
  }
  // TODO
  // 特性
  // virtual int UseAbility(const CostPtr cost) = 0;
  // // 招式（不止一个）
 private:
  card::common::EnergyType energy_type;
  card::common::ElementalType elemental_type;
};

}