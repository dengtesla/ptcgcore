#include <memory>

#include "stage.h"

class World {
 public:
  int GetStage(int player_id, const StagePtr stage);

 private:
  int go_first_player_id_;
  StagePtr stage_first_;
  StagePtr stage_second_;
};

using WorldPtr = std::shared_ptr<World>;