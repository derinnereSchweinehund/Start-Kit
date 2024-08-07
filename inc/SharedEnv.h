#ifndef SHAREDENV_H
#define SHAREDENV_H

#include "ActionModel.h"
#include "States.h"
#include "Status.hpp"
#include "Tasks.h"

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
      : num_of_agents_(num_of_agents), timestep_(0) {
    current_states_.resize(num_of_agents);
    assigned_tasks_.resize(num_of_agents);
    assigned_actions_.resize(num_of_agents);
    current_status_.resize(num_of_agents);
  }
  vector<int> goal_locations(int agent_num){
    
  }
};

#endif
