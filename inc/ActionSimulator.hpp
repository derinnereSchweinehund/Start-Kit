#include "ActionModel.h"
#include "SharedEnv.h"
#include "Status.hpp"
#include <boost/asio/io_service.hpp>
#include <random>

template <class ActionModel> class ActionSimulator {
public:
  ActionSimulator(ActionModel &model, SharedEnvironment *env)
      : model(model), env(env){};
  virtual vector<Status> simulate_action(vector<Action> &next_actions);
  virtual bool validate_safe(const vector<Action> &next_actions);

protected:
  ActionModel &model;
  SharedEnvironment *env;

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

};

// Classical MAPF scenario where all actions succeed
// I wanted to write a template <class T> to do ActionSimulator<T>  
class PerfectSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  PerfectSimulator(ActionModelWithRotate &model, SharedEnvironment *env)
      : ActionSimulator(model, env){};

  vector<Status> simulate_action(vector<Action> &next_actions) override;

  bool validate_safe(const vector<Action> &next_actions) override;

};

// Implements delay probability for MAPF-DP
class ProbabilisticSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  ProbabilisticSimulator(float success_chance, ActionModelWithRotate &model,
                         SharedEnvironment *env)
      : success_chance_(success_chance), rd_(), gen_(rd_()), distrib_(0, 1),
        ActionSimulator(model, env){};

  vector<Status> simulate_action(vector<Action> &next_actions) override;

  virtual bool validate_safe(const vector<Action> &next_actions) override;

private:
  float success_chance_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> distrib_;
};

bool validate_unfailing(const vector<Action> &next_actions,
                        const SharedEnvironment *env,
                        ActionModelWithRotate &model);

bool validate_failing(const vector<Action> &next_actions,
                      const SharedEnvironment *env,
                      ActionModelWithRotate &model);