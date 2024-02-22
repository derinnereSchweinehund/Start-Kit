#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include "SharedEnv.h"
#include "Tasks.h"
#include "timer.h"
#include <algorithm>
#include <cstdint>
#include <fstream>

namespace task_generator {

class TaskGenerator {
public:
  TaskGenerator(std::ifstream &istream)
      : all_tasks_(), timer_(), num_revealed_tasks_(0), reveal_interval_(1) {

    size_t num_of_tasks = istream.get();
    for (size_t i = 0; i < num_of_tasks; i++) {
      uint32_t location = istream.get();
      all_tasks_.emplace_back(i, location);
    }
  }

  bool update_task(SharedEnvironment &state) {

    // - check current positions and tick off completed task.
    for (size_t i = 0; i < state.num_of_agents_; i++) {
      if (state.current_states_[i].location ==
          state.assigned_tasks_[i].front().location) {
        state.assigned_tasks_[i].pop_front();
      }
    }

    if (tasks_remaining(state)) {
      return false;
    }

    if (timer_.elapsed_time_sec() < reveal_interval_) {
      return true;
    }
    // - check reveal times of hidden tasks.
    size_t to_reveal =
        std::min(num_revealed_tasks_ + max_to_reveal_, all_tasks_.size());
    for (; num_revealed_tasks_ < to_reveal; num_revealed_tasks_++) {
      state.available_tasks_.push_back(all_tasks_[num_revealed_tasks_]);
    }

    timer_.start();
    return true;
  }

private:
  std::vector<tasks::Task> all_tasks_;
  Timer timer_;
  size_t num_revealed_tasks_;
  double reveal_interval_;
  size_t max_to_reveal_ = 5;

  bool tasks_remaining(SharedEnvironment &state) {

    if (state.available_tasks_.empty()) {
      return true;
    }

    for (size_t i = 0; i < state.num_of_agents_; i++) {
      if (!state.assigned_tasks_[i].empty()) {
        return true;
      }
    }

    return num_revealed_tasks_ < all_tasks_.size();
  }
};
} // namespace task_generator

#endif
