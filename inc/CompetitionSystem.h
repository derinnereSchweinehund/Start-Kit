#pragma once
#include "ActionModel.h"
#include "Logger.h"
#include "SharedEnv.h"
#include <memory>

namespace base_system {

struct metrics_t {
  std::vector<vector<Action>> actual_movements;
  std::vector<vector<Action>> planner_movements;
  std::vector<Path> paths;
  std::vector<size_t> solution_costs;

  metrics_t()  {}
};

template <class Task_Generator, class Task_Assigner, class Execution_Policy,
          class Planner, class Simulator>
class BaseSystem {
public:
  BaseSystem(Task_Generator *task_generator, Task_Assigner *task_assigner,
             Execution_Policy *execution_policy, Planner *planner,
             Simulator *simulator, Logger *logger)
      : task_generator_(task_generator), task_assigner_(task_assigner),
        execution_policy_(execution_policy), planner_(planner),
        simulator_(simulator), logger_(logger) {}

  //void simulate(SharedEnvironment *state, int simulation_time);
  void simulate(SharedEnvironment *state, int simulation_time) {

    // immutable state pointer for those pesky user defined functions
    //const SharedEnvironment *const immutable_state = state;
    //consider making state a shared_ptr
    std::cout << task_generator_->update_tasks(*state) << '\n';

    while (!task_generator_->update_tasks(*state)) {
      std::cout << "HI T" << std::flush;
      task_assigner_->assign_tasks(state);
      std::cout << "1" << std::flush;
      std::vector<Action> next_actions =
          execution_policy_->get_actions(state);
      std::cout << "2" << std::flush;
      simulator_->simulate_actions(*state, next_actions);
    }
}
  const metrics_t &get_metrics() const { return metrics_; }

private:
  Task_Generator *task_generator_;
  Task_Assigner *task_assigner_;
  Execution_Policy *execution_policy_;
  Planner *planner_;
  Simulator *simulator_;

  Logger *logger_;
  metrics_t metrics_;
};

} // namespace base_system
