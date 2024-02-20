#ifndef TASK_ASSIGNER_H
#define TASK_ASSIGNER_H

#include "SharedEnv.h"
#include "Tasks.h"

namespace task_assigner {
class TaskAssigner {
public:
  TaskAssigner() {}

  std::vector<Task> assign_tasks(const SharedEnvironment *const state) {
    return std::vector<Task>();
  }
};

} // namespace task_assigner

#endif
