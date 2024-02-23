#include "ActionModel.h"
#include "ScheduleTable.hpp"
#include "SharedEnv.h"
#include "planner_wrapper.h"
#include <vector>

namespace execution_policy {

template <class P> class ExecutionPolicy {

public:
  ExecutionPolicy(P *planner) : planner_(planner_) {}

  std::vector<Action> &get_actions(const SharedEnvironment *state) {
    return planner_->query(state->current_states_, state->assigned_tasks_, 3.0);
  }

private:
  P *const planner_;
};

typedef ExecutionPolicy<planner::MAPFPlannerWrapper> MAPFExecutionPolicy;

} // namespace execution_policy

// by queuing reservations on each location
// Using the ActionModelWithRotate to predict result states of actions
template <class P> class MinimumCommunicationPolicy {
public:
  MinimumCommunicationPolicy(P *planner, int buffer_size)
      : planner_(planner), schedule_() {}

  // Delay action if the agent is waiting for another to use the next location
  std::vector<Action> get_actions(const SharedEnvironment *state) {
    schedule_.clear();

    std::vector<int> goal_locations(state->num_of_agents_);
    for (size_t i = 0; i < state->num_of_agents_; i++) {
      goal_locations[i] = state->assigned_tasks_[i].front().location;
    }
    vector<Action> planner_actions =
        planner_->query(state->current_states_, goal_locations);
    vector<State> next_states =
        planner_model_.result_states(state->current_states_, planner_actions);
    schedule_.insert_step(next_states);

    vector<Action> corrected_actions(state->num_of_agents_);
    for (int i = 0; i < state->num_of_agents_; i++) {
      if (schedule_.is_scheduled(i, next_states[i].location)) {
        corrected_actions[i] = planner_actions[i];
      } else {
        corrected_actions[i] = Action::W;
      }
    }
    return corrected_actions;
  }

private:
  P *const planner_;
  ScheduleTable schedule_;
  ActionModelWithRotate planner_model_;
};
