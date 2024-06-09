#ifndef SHAREDENV_H
#define SHAREDENV_H

#include "ActionModel.h"
#include "States.h"
#include "Status.hpp"
#include "Tasks.h"

namespace state {

struct SharedEnvironment {
  size_t num_of_agents_;
  size_t timestep_;

  // list of all available_tasks
  std::vector<tasks::Task> available_tasks_;

  std::vector<State> current_states_;
  std::vector<deque<tasks::Task>> assigned_tasks_;
  std::vector<Action> assigned_actions_;
  std::vector<Status> current_status_;

  SharedEnvironment(size_t num_of_agents)
      : num_of_agents_(num_of_agents), timestep_(0),
        available_tasks_(num_of_agents_), current_states_(num_of_agents_),
        assigned_tasks_(num_of_agents_), assigned_actions_(num_of_agents_),
        current_status_(num_of_agents_) {}
};

#endif
} // namespace state
