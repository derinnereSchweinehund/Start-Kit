#include "ActionSimulator.hpp"
#include "ActionModel.h"
#include "SharedEnv.h"
#include <vector>

void PerfectSimulator::simulate_action(SharedEnvironment &state,
                                       const vector<Action> &next_actions) {
  if (!validate_safe(state, next_actions)) {
    for (size_t i = 0; i < state.num_of_agents_; i++) {
      state.current_status_[i] = Status::FAILED;
    }
    return;
  }
  state.current_states_ =
      model.result_states(state.current_states_, next_actions);
  for (size_t i = 0; i < state.num_of_agents_; i++) {
    state.current_status_[i] = Status::SUCCEEDED;
  }
  return;
}

bool PerfectSimulator::validate_safe(const SharedEnvironment &state,
                                     const vector<Action> &next_actions) {
  return validate_unfailing(next_actions, &state, model);
}

bool ProbabilisticSimulator::validate_safe(const SharedEnvironment &state,
                                           const vector<Action> &next_actions) {
  return validate_failing(next_actions, &state, model);
}

void ProbabilisticSimulator::simulate_action(
    SharedEnvironment &state, const vector<Action> &next_actions) {
  if (!validate_safe(state, next_actions)) {
    for (size_t i = 0; i < state.num_of_agents_; i++) {
      state.current_status_[i] = Status::FAILED;
    }
    return;
  }

  vector<Status> progress(state.num_of_agents_);
  for (int i = 0; i < state.num_of_agents_; i++) {
    // Succeeds success_chance % of the time, never if <=0 and always if >=1
    if (success_chance_ > distrib_(gen_)) {
      state.current_status_[i] = Status::SUCCEEDED;
      state.current_states_[i] =
          model.result_state(state.current_states_[i], next_actions.at(i));
    } else {
      state.current_status_[i] = Status::FAILED;
    }
  }
  return;
}
