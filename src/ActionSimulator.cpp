#include "ActionSimulator.hpp"
#include "ActionModel.h"
#include "SharedEnv.h"
#include <unordered_set>
#include <vector>

// Check for vertex and edge conflicts
bool validate_unfailing(const vector<Action> &next_actions,
                        const SharedEnvironment *env,
                        ActionModelWithRotate &model) {
  vector<State> next_states =
      model.result_states(env->curr_states, next_actions);
  // Check vertex conflicts
  for (int i = 0; i < env->num_of_agents; i++) {
    for (int j = i + 1; j < env->num_of_agents; j++) {
      if (next_states.at(i).location == next_states.at(j).location) {
        return false;
      }
    }
  }
  // Check for edge conflicts
  // If current and next state coincide in direction
  for (int i = 0; i < env->num_of_agents; i++) {
    for (int j = 0; j < env->num_of_agents; i++) {
      if (next_states.at(i).location == env->curr_states.at(j).location &&
          next_states.at(j).location == env->curr_states.at(i).location) {
        return false;
      }
    }
  }
  return true;
}

bool validate_failing(const vector<Action> &next_actions,
                      const SharedEnvironment *env,
                      ActionModelWithRotate &model) {
  // Check for vertex and edge conflicts
  assert(next_actions.size() == env->num_of_agents);
  if (!validate_unfailing(next_actions, env, model)) {
    return false;
  }
  // Check for 1-robustness
  unordered_set<int> occupied_before;
  const vector<State> &curr_states = env->curr_states;
  for (int i = 0; i < env->num_of_agents; i++) {
    occupied_before.insert({curr_states[i].location});
  }
  vector<State> next_states = model.result_states(curr_states, next_actions);
  for (int i = 0; i < env->num_of_agents; i++) {
    auto res = occupied_before.find(next_states[i].location);
    if (res != occupied_before.end()) {
      return false;
    }
  }

  return true;
}

vector<Status> PerfectSimulator::simulate_action(vector<Action> &next_actions) {
  if (!validate_safe(next_actions)) {
    return vector<Status>(env->num_of_agents, Status::FAILED);
  }
  env->curr_states = model.result_states(env->curr_states, next_actions);
  return vector<Status>(env->num_of_agents, Status::SUCCESS);
}

bool PerfectSimulator::validate_safe(const vector<Action> &next_actions) {
  return validate_unfailing(next_actions, env, model);
}

bool ProbabilisticSimulator::validate_safe(const vector<Action> &next_actions) {
  return validate_failing(next_actions, env, model);
}

vector<Status>
ProbabilisticSimulator::simulate_action(vector<Action> &next_actions) {
  if (!validate_safe(next_actions)) {
    return vector<Status>(env->num_of_agents, Status::FAILED);
  }

  vector<Status> progress(env->num_of_agents);
  for (int i = 0; i < env->num_of_agents; i++) {
    // Succeeds success_chance % of the time, never if <=0 and always if >=1
    if (success_chance_ > distrib_(gen_)) {
      progress[i] = Status::SUCCESS;
      State &curr = env->curr_states.at(i);
      curr = model.result_state(curr, next_actions.at(i));
    } else {
      progress[i] = Status::FAILED;
    }
  }
  return progress;
}