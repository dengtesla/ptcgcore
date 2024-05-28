#pragma once

#include "card/card.h"
#include "card/monster_card.h"
#include "card/energy_card.h"
#include "card/trainer_card.h"
#include "common/file.h"

#include "card.pb.h"
#include "card/monster_card.pb.h"
#include "card/energy_card.pb.h"
#include "card/trainer_card.pb.h"

#include "spdlog/spdlog.h"

namespace ptcgcore {

class CardFactory {
 public:
  CardPtr CreateCard(const std::string& card_name);
  CardPtr CreateCard(const card::Card& card_msg);

  static std::shared_ptr<CardFactory> Instance();
 private:
  CardFactory() {
    card::CardDataset dataset;
    file::GetProto("../card/impl/card_dataset.pb.txt", dataset);
    spdlog::debug("card_dataset: {}", dataset.DebugString());
    for (const auto& card : dataset.card()) {
      const auto& name = card.name();
      if (card_repo_.find(name) == card_repo_.end()) {
        spdlog::info("register card: {}", name);
        card_repo_[name] = card;
      } else {
        spdlog::error(name + " already exist: " + card_repo_.at(name).DebugString());
      }
    }
  }
  // ~CardFactory() = default;
  CardFactory(const CardFactory& other) = delete;
  CardFactory& operator=(const CardFactory&other) = delete;
  static std::shared_ptr<CardFactory> instance;
  static std::mutex m_mutex;
  static std::unordered_map<std::string, card::Card> card_repo_;
};


}  // namespace ptcgcore