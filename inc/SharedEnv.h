#pragma once
#include "ActionModel.h"
#include "Grid.h"
#include "States.h"
#include "Status.hpp"
#include "Tasks.h"
#include "nlohmann/json.hpp"

struct SharedEnvironment {
  size_t num_of_agents_;
  size_t timestep_;
  // list of all available_tasks
  std::vector<Task> available_tasks_;

  std::vector<State> current_states_;
  std::vector<deque<Task>> assigned_tasks_;
  std::vector<Action> assigned_actions_;
  std::vector<Status> current_status_;

  SharedEnvironment(size_t num_of_agents)
      : num_of_agents_(num_of_agents), timestep_(0) {
    current_states_.resize(num_of_agents);
    assigned_tasks_.resize(num_of_agents);
    assigned_actions_.resize(num_of_agents);
    current_status_.resize(num_of_agents);
  }
};
