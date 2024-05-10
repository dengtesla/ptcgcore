#pragma once

#include "card/card.h"

namespace ptcgcore {

class ICardPlace {
  enum Type {
    ACTIVE = 0,
    BENCHED = 1,
    DECK = 2,
    DISCARD = 3,
    LOST_ZONE = 4,
    STADIUM = 5
  };
};


// struct SingleFunc {
//   enum Type {
//     HOW_MANY = 1,
//   };
//   enum Place {
//     HEAD = 1,
//     DECK = 2,
//     DISCARD = 3,
//   };

//   Type func_type;
//   Place from_place;
//   Place target_place;
// }

class Func {
  // 满足 condition 和 cost 的支付能力才能发动
  // 支付 cost 后进行 effects 的结算
  std::vector<Cost> costs;
  std::vector<Condition> condition;
  std::vector<Effect> effects;

  const CardPtr func_card; // 发动这个效果的主体是哪张卡
}

class MonsterPile : public ICardPlace {
  std::vector<CardPtr> monsters;
  std::vector<CardPtr> energys;
  std::vector<CardPtr> items; // 一般是唯一的

  int damage_counters = 0; // 伤害指示物

  // TODO:

  // int GetUsableFunc(std::vector<void (*)()> functions);

  int GetAllFunc(std::vector<Func>);

};

class Stadium : public ICardPlace {
  CardPtr stadium_card;
};

}