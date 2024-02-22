#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include "SharedEnv.h"
#include "Tasks.h"
#include "timer.h"
#include <cstdint>
#include <fstream>

namespace task_generator {

class TaskGenerator {
public:
  TaskGenerator(std::ifstream &istream)
      : all_tasks_(), timer_(), num_revealed_tasks(0), reveal_interval_(1) {

    size_t num_of_tasks = istream.get();
    for (size_t i = 0; i < num_of_tasks; i++) {
      uint32_t location = istream.get();
      all_tasks_.emplace_back(i, location);
    }
  }

  bool update_task(SharedEnvironment *state) {

    // TODO: mutate current state
    // - check current positions and tick off completed task.
    if (tasks_completed()) {
      return false;
    }

    if (timer_.elapsed_time_sec() < reveal_interval_) {
      return true;
    }
    // - check reveal times of hidden tasks.
    for (size_t i = num_revealed_tasks; i < all_tasks_.size();
         i++, num_revealed_tasks++) {
      state->available_tasks_.push_back(all_tasks_[i]);
    }

    timer_.start();
    return true;
  }

private:
  std::vector<tasks::Task> all_tasks_;
  Timer timer_;
  size_t num_revealed_tasks;
  double reveal_interval_;

  bool tasks_completed() { return false; }
};
} // namespace task_generator

#endif
