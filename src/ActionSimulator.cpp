#include "ActionSimulator.hpp"
#include "ActionModel.h"
#include "SharedEnv.h"
#include <vector>


vector<Status> PerfectSimulator::simulate_action(vector<Action> &next_actions) {
  if (!validate_safe(next_actions)) {
    return vector<Status>(env->num_of_agents, Status::FAILED);
  }
  env->curr_states = model.result_states(env->curr_states, next_actions);
  return vector<Status>(env->num_of_agents, Status::SUCCEEDED);
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
      progress[i] = Status::SUCCEEDED;
      State &curr = env->curr_states.at(i);
      curr = model.result_state(curr, next_actions.at(i));
    } else {
      progress[i] = Status::FAILED;
    }
  }
  return progress;
}