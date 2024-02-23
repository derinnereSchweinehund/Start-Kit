#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include "SharedEnv.h"
#include "timer.h"
#include <cstddef>
#include <fstream>

namespace task_generator {

struct task_generator_metrics_t {
  size_t num_of_task_finish;
};

class TaskGenerator {
public:
  TaskGenerator(std::string task_file)
      : all_tasks_(), timer_(), num_revealed_tasks_(0), reveal_interval_(10),
        max_to_reveal_(5) {

    std::ifstream task_file_stream(task_file.c_str());
    size_t num_of_tasks = task_file_stream.get();
    for (size_t i = 0; i < num_of_tasks; i++) {
      uint32_t location = task_file_stream.get();
      all_tasks_.emplace_back(i, location);
    }
    task_file_stream.close();
  }

  // - update the task generator state
  // - return true if the task generator has no more tasks to reveal
  bool update_tasks(SharedEnvironment * state) {

    for (size_t i = 0; i < state->num_of_agents_; i++) {
      if (state->current_states_[i].location ==
          state->assigned_tasks_[i].front().location) {
        state->assigned_tasks_[i].pop_front();
      }
    }

    if (!tasks_remaining(state)) {
      return false;
    }

    if (timer_.elapsed_time_sec() < reveal_interval_) {
      return true;
    }

    // - reveal next batch of tasks.
    size_t to_reveal =
        std::min(num_revealed_tasks_ + max_to_reveal_, all_tasks_.size());
    for (; num_revealed_tasks_ < to_reveal; num_revealed_tasks_++) {
      state->available_tasks_.push_back(all_tasks_[num_revealed_tasks_]);
    }

    timer_.start();
    return true;
  }

  const task_generator_metrics_t &get_metrics() { return metrics_; }

private:
  std::vector<tasks::Task> all_tasks_;
  size_t num_revealed_tasks_;
  double reveal_interval_;
  size_t max_to_reveal_;
  task_generator_metrics_t metrics_;
  Timer timer_;

  bool tasks_remaining(SharedEnvironment *state) {

    if (state->available_tasks_.empty()) {
      return true;
    }

    for (size_t i = 0; i < state->num_of_agents_; i++) {
      if (!state->assigned_tasks_[i].empty()) {
        return true;
      }
    }

    return num_revealed_tasks_ < all_tasks_.size();
  }
};
} // namespace task_generator

#endif
