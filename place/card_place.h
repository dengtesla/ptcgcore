namespace ptcgcore {

class ICardPlace {
  enum Type {
    ACTIVE = 0;
    BENCHED = 1;
    DECK = 2;
    DISCARD = 3;
    LOST_ZONE = 4;
    STADIUM = 5;
  }
};


class MonsterPile : public ICardPlace {
  std::vector<CardPtr> monsters;
  std::vector<CardPtr> energys;
  std::vector<CardPtr> items; // 一般是唯一的

  int damage_counters = 0; // 伤害指示物

  // TODO: 

};

class Stadium : public ICardPlace {
  CardPtr stadium_card;
};

}