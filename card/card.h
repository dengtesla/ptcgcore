#include <memory>
#include <string>
#include <vector>

#include "buff/buff.h"
#include "world.h"
#include "cost/cost.h"

enum CardType {
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

class ICard {
 public:
  ICard(std::string &name) : name_(name) {}
  ~ICard() = default;
  std::string GetName() { return name_; };
  int use_ability(const CostPtr cost, WorldPtr world);
 private:
  std::string name_ = "";
  std::vector<SpecialRule> rule_;
  std::vector<IBuff> buff_;  // 该卡片此时的 buff
};

using CardPtr = std::shared_ptr<ICard>;
