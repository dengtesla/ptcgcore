#include <memory>
#include <vector>

#include "card/card.h"
#include "buff/buff.h"
#include "cost/cost.h"

namespace ptcgcore {

class Stage {
 public:
  int GetPlayerId();
  int GetBench(std::vector<CardPtr>& bench) const;
  int GetHand(std::vector<CardPtr>& hand) const;
  int GetLostZone(std::vector<CardPtr>& lost_zone) const;
  int GetDeck(std::vector<CardPtr>& deck, bool hidden = true) const;
  int GetPrizeCard(std::vector<CardPtr>& prize_card, bool hidden = true) const;

  int ShuffleDeck();
  int ShufflePrize();
  int DrawOneCard();
  int ProcessCost(const Cost);
 private:
  int player_id = -1;
  CardPtr active_pose; // 战斗场
  std::vector<CardPtr> bench_; // 备战区
  std::vector<CardPtr> deck_; // 卡组
  std::vector<CardPtr> prize_card_; // 奖赏卡
  std::vector<CardPtr> hand_; // 手牌
  std::vector<CardPtr> lost_zone_; // 放逐区

  std::vector<IBuff> buff_; // 对玩家的 buff
};

using StagePtr = std::shared_ptr<Stage>;
using StageConstPtr = std::shared_ptr<const Stage>;

}
