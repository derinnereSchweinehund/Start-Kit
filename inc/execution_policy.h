#include "ActionModel.h"
#include "SharedEnv.h"
#include "planner_wrapper.h"
#include <vector>
#include "ScheduleTable.hpp"

template <class P> class ExecutionPolicy {

public:
  ExecutionPolicy(P *planner) : planner_(planner) {}

  std::vector<Action> &get_actions(const SharedEnvironment &env) {
    return planner_->query(env.curr_states, env.goal_locations);
  }

private:
  const P *planner_;
};


// MinimumCommunicationPolicy maintains a partial ordering of path dependencies
// by queuing reservations on each location
// Using the ActionModelWithRotate to predict result states of actions
template <class P> class MinimumCommunicationPolicy {

public:
  MinimumCommunicationPolicy(P *planner, int buffer_size) :
  planner_(planner), schedule_(){}

  // Delay action if the agent is waiting for another to use the next location
  std::vector<Action> get_actions(const SharedEnvironment &env) {
    schedule_.clear();
    vector<Action> planner_actions = planner_->query(env.curr_states, env.goal_locations);
    vector<State> next_states = planner_model_.result_states(env.curr_states, planner_actions);
    schedule_.insert_step(next_states);

    vector<Action> corrected_actions(env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; i++) {
      if (schedule_.is_scheduled(i, next_states[i].location)) {
        corrected_actions[i] = planner_actions[i];
      } else {
        corrected_actions[i] = Action::W;
      }
    }
    return corrected_actions;
  }
private:
  P* const planner_;
  ScheduleTable schedule_;
  ActionModelWithRotate planner_model_;

};