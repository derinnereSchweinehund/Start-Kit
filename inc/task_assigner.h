#ifndef TASK_ASSIGNER_H
#define TASK_ASSIGNER_H

#include "SharedEnv.h"
#include "Tasks.h"

namespace task_assigner {

class TaskAssigner {

public:
  TaskAssigner() : agent_id_(0) {}

  std::vector<deque<tasks::Task>> assign_tasks(SharedEnvironment *state) {

    // Assign tasks to agents in a round robin fashion
    std::vector<deque<tasks::Task>> assigned_tasks(state->num_of_agents_);
    for (size_t task_id; task_id < state->available_tasks_.size(); task_id++) {
      assigned_tasks[agent_id_].push_back(state->available_tasks_[task_id]);
      agent_id_ = (agent_id_++) % state->num_of_agents_;
    }
    return assigned_tasks;
  }

private:
  size_t agent_id_;
};

} // namespace task_assigner

#endif
