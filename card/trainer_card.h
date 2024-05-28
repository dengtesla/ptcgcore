#pragma once

#include <memory>

#include "card/card.h"
#include "cost/cost.h"
#include "world.h"

#include "card.pb.h"

namespace ptcgcore {

class TrainerCard : public ICard{
 public:
  TrainerCard(const std::string& name) : ICard(name) {
    card_type_ = card::common::BasicCardType::TRAINER;
  }
  TrainerCard(const card::Card& card) : ICard(card) {
    
  }
  // TODO
  // 特性
  // virtual int UseAbility(const CostPtr cost) = 0;
  // // 招式（不止一个）
 private:

};

}