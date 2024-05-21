#pragma once

namespace ptcgcore {

class Cost {
};

class CardCost : public Cost {
  enum Type {
    SEND = 1,
    SELECT = 2,
  };
  enum CardNum {
    EXACT_NUM = 1,
    ANY_NUM = 2,
    ANY_POSITIVE_NUM = 3,
    AS_MANY_AS_YOU_CAN = 4,
    UP_TO = 5,
  };
  enum CardType {
    ANY = 0,
    ITEM = 1,
    // energy
    ANY_ENERGY = 100,
    ANY_BASIC_ENERGY = 101,
    ANY_SPECIAL_ENERGY = 102,
    FIRE_ENERGY = 103,
  };
  enum Place {
    NONE = 0,
    DECK = 1,
    HAND = 2,
    DISCARD = 3,
    BENCHED = 4,
    ACTIVE = 5,
    LOST_ZONE = 6,
  };

  Type type;
  CardType card_type;
  Place from_place;
  Place Target_place;
  CardNum card_num;
  int num;
};

class CoinCost : public Cost {
  int num;
};

/*
sample:
 将一张手牌送入墓地发动:
  type: HOW_MANY
  from_place: HAND
  target_place: DISCARD
  card_type: ANY
*/

using CostPtr = std::shared_ptr<Cost>;

}
