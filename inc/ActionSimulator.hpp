#ifndef ACTION_SIMULATOR_H
#define ACTION_SIMULATOR_H

#include "ActionModel.h"
#include "SharedEnv.h"
#include <random>
#include <vector>

template <class ActionModel> class ActionSimulator {
public:
  ActionSimulator(ActionModel &model) : model_(model){};
  virtual void simulate_action(SharedEnvironment *state,
                               const vector<Action> &next_actions);
  virtual bool validate_safe(const SharedEnvironment *state,
                             const vector<Action> &next_actions);

protected:
  ActionModel &model_;
};

// Classical MAPF scenario where all actions succeed
// I wanted to write a template <class T> to do ActionSimulator<T>
template <class ActionModel> class PerfectSimulator {
private:
  ActionModel &model_;

public:
  PerfectSimulator(ActionModelWithRotate &model) : model_(model){};

  void simulate_action(SharedEnvironment *state,
                       const vector<Action> &next_actions) {
    if (!validate_safe(state, next_actions)) {
      for (size_t i = 0; i < state->num_of_agents_; i++) {
        state->current_status_[i] = Status::FAILED;
      }
    }
    state->current_states_ =
        this->model_.result_states(state->current_states_, next_actions);
    for (size_t i = 0; i < state->num_of_agents_; i++) {
      state->current_status_[i] = Status::SUCCESS;
    }
  }
  bool validate_safe(const SharedEnvironment *state,
                     const vector<Action> &next_actions) {
    return this->validate_unfailing(next_actions, state);
  }

  // Check for vertex and edge conflicts
  bool validate_unfailing(const vector<Action> &next_actions,
                          const SharedEnvironment *state) {
    vector<State> next_states =
        model_.result_states(state->current_states_, next_actions);
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
                        const SharedEnvironment *state) {
    // Check for vertex and edge conflicts
    assert(next_actions.size() == state->num_of_agents_);
    if (!validate_unfailing(next_actions, state, model_)) {
      return false;
    }
    // Check for 1-robustness
    unordered_set<int> occupied_before;
    const vector<State> &current_states_ = state->current_states_;
    for (int i = 0; i < state->num_of_agents_; i++) {
      occupied_before.insert({current_states_[i].location});
    }
    vector<State> next_states =
        model_.result_states(current_states_, next_actions);
    for (int i = 0; i < state->num_of_agents_; i++) {
      auto res = occupied_before.find(next_states[i].location);
      if (res != occupied_before.end()) {
        return false;
      }
    }

    return true;
  }
};

// Implements delay probability for MAPF-DP
template <class ActionModel> class ProbabilisticSimulator {
private:
  ActionModel &model_;

public:
  ProbabilisticSimulator(float success_chance, ActionModelWithRotate &model)
      : success_chance_(success_chance), rd_(), gen_(rd_()), distrib_(0, 1) {
      }

    bool validate_safe(const SharedEnvironment *state,
                       const vector<Action> &next_actions)  {
      return this->validate_failing(next_actions, state);
    }
    void simulate_action(SharedEnvironment * state,
                         const vector<Action> &next_actions)  {
      if (!validate_safe(state, next_actions)) {
        for (size_t i = 0; i < state->num_of_agents_; i++) {
          state->current_status_[i] = Status::FAILED;
        }
      }

      vector<Status> progress(state->num_of_agents_);
      for (int i = 0; i < state->num_of_agents_; i++) {
        // Succeeds success_chance % of the time, never if <=0 and always if >=1
        if (success_chance_ > distrib_(gen_)) {
          state->current_status_[i] = Status::SUCCESS;
          state->current_states_[i] = this->model_.result_state(
              state->current_states_[i], next_actions.at(i));
        } else {
          state->current_status_[i] = Status::FAILED;
        }
      }
    }

    // Check for vertex and edge conflicts
    bool validate_unfailing(const vector<Action> &next_actions,
                            const SharedEnvironment *state) {
      vector<State> next_states =
          model_.result_states(state->current_states_, next_actions);
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
                          const SharedEnvironment *state) {
      // Check for vertex and edge conflicts
      assert(next_actions.size() == state->num_of_agents_);
      if (!validate_unfailing(next_actions, state, model_)) {
        return false;
      }
      // Check for 1-robustness
      unordered_set<int> occupied_before;
      const vector<State> &current_states_ = state->current_states_;
      for (int i = 0; i < state->num_of_agents_; i++) {
        occupied_before.insert({current_states_[i].location});
      }
      vector<State> next_states =
          model_.result_states(current_states_, next_actions);
      for (int i = 0; i < state->num_of_agents_; i++) {
        auto res = occupied_before.find(next_states[i].location);
        if (res != occupied_before.end()) {
          return false;
        }
      }

      return true;
    }

  private:
    float success_chance_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> distrib_;
  };

#endif
