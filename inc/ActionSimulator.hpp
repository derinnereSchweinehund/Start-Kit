#include "ActionModel.h"
#include "SharedEnv.h"
#include <random>

template <class ActionModel> class ActionSimulator {
public:
  ActionSimulator(ActionModel &model) : model(model){};
  virtual void simulate_action(SharedEnvironment &state,
                               const vector<Action> &next_actions);
  virtual bool validate_safe(const SharedEnvironment &state,
                             const vector<Action> &next_actions);

protected:
  ActionModel &model;
  SharedEnvironment *state;

  // Check for vertex and edge conflicts
  bool validate_unfailing(const vector<Action> &next_actions,
                          const SharedEnvironment *state,
                          ActionModelWithRotate &model) {
    vector<State> next_states =
        model.result_states(state->current_states_, next_actions);
    // Check vertex conflicts
    for (int i = 0; i < state->num_of_agents_; i++) {
      for (int j = i + 1; j < state->num_of_agents_; j++) {
        if (next_states.at(i).location == next_states.at(j).location) {
          return false;
        }
      }
    }
    // Check for edge conflicts
    // If current and next state coincide in direction
    for (int i = 0; i < state->num_of_agents_; i++) {
      for (int j = 0; j < state->num_of_agents_; i++) {
        if (next_states.at(i).location ==
                state->current_states_.at(j).location &&
            next_states.at(j).location ==
                state->current_states_.at(i).location) {
          return false;
        }
      }
    }
    return true;
  }

  bool validate_failing(const vector<Action> &next_actions,
                        const SharedEnvironment *state,
                        ActionModelWithRotate &model) {
    // Check for vertex and edge conflicts
    assert(next_actions.size() == state->num_of_agents_);
    if (!validate_unfailing(next_actions, state, model)) {
      return false;
    }
    // Check for 1-robustness
    unordered_set<int> occupied_before;
    const vector<State> &current_states_ = state->current_states_;
    for (int i = 0; i < state->num_of_agents_; i++) {
      occupied_before.insert({current_states_[i].location});
    }
    vector<State> next_states =
        model.result_states(current_states_, next_actions);
    for (int i = 0; i < state->num_of_agents_; i++) {
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
  PerfectSimulator(ActionModelWithRotate &model) : ActionSimulator(model){};

  void simulate_action(SharedEnvironment &state,
                       const vector<Action> &next_actions) override;

  bool validate_safe(const SharedEnvironment &state,
                     const vector<Action> &next_actions) override;
};

// Implements delay probability for MAPF-DP
class ProbabilisticSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  ProbabilisticSimulator(float success_chance, ActionModelWithRotate &model)
      : success_chance_(success_chance), rd_(), gen_(rd_()), distrib_(0, 1),
        ActionSimulator(model){};

  void simulate_action(SharedEnvironment &state,
                       const vector<Action> &next_actions) override;

  virtual bool validate_safe(const SharedEnvironment &state,
                             const vector<Action> &next_actions) override;

private:
  float success_chance_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> distrib_;
};

bool validate_unfailing(const vector<Action> &next_actions,
                        const SharedEnvironment *state,
                        ActionModelWithRotate &model);

bool validate_failing(const vector<Action> &next_actions,
                      const SharedEnvironment *state,
                      ActionModelWithRotate &model);
