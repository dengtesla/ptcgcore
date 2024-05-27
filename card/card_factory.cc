#include "card/card_factory.h"
#include "card/monster_card.h"

namespace ptcgcore {

std::shared_ptr<CardFactory> CardFactory::Instance() {
  if (instance == nullptr) {
    std::lock_guard<std::mutex> lk(m_mutex);
  }
  if (instance == nullptr) {
    instance = std::shared_ptr<CardFactory>(new CardFactory());
  }
  return instance;
}

CardPtr CardFactory::CreateCard(const std::string& card_name) {
  if (card_repo_.find(card_name) == card_repo_.end()) {
    spdlog::error("card not found! please check, card_name: {}", card_name);
    return std::make_shared<ICard>("ERROR");
  }
  const auto& card = card_repo_.at(card_name);
  if (card.type() == card::common::BasicCardType::MONSTER) {
    return std::make_shared<MonsterCard>(card);
  } else if (card.type() == card::common::BasicCardType::TRAINER) {
    return std::make_shared<ICard>(card);
  } else if (card.type() == card::common::BasicCardType::ENERGY) {
    return std::make_shared<ICard>(card);
  }
  spdlog::error("card type not exist! card: {}", card.DebugString());
  return std::make_shared<ICard>("ERROR");
}

std::shared_ptr<CardFactory> CardFactory::instance = nullptr;
std::mutex CardFactory::m_mutex;


}  // namespace ptcgcore