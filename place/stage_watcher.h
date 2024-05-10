#pragma once

#include <memory>
#include <mutex>

#include "place/stage.h"

namespace ptcgcore {

class StageWatcher {
 private:
  StageWatcher(StageConstPtr stage_first, StageConstPtr stage_second):
      stage_first_(stage_first), stage_second_(stage_second) {};
  StageWatcher(const StageWatcher&);
  StageWatcher& operator =(const StageWatcher&);
 public:
  static std::shared_ptr<StageWatcher> getInstance(
    StageConstPtr stage_first, StageConstPtr stage_second) {
    if (instance == nullptr) {
      std::lock_guard<std::mutex> lk(m_mutex);
    }
    if (instance == nullptr) {
      instance = std::shared_ptr<StageWatcher>(new StageWatcher(stage_first, stage_second));
    }
    return instance;
  }

  static std::shared_ptr<StageWatcher> getInstance() {
    return instance;
  }

  ~StageWatcher() {};
 private:
  static std::shared_ptr<StageWatcher> instance;
  static std::mutex m_mutex;
  StageConstPtr stage_first_;
  StageConstPtr stage_second_;
};

} //  namespace ptcgcore