#ifndef TASK_ASSIGNER_H
#define TASK_ASSIGNER_H

#include "SharedEnv.h"
#include "Tasks.h"

namespace task_assigner {

class TaskAssigner {

public:
  TaskAssigner() {}

  std::vector<Task> assign_tasks(const SharedEnvironment *const state) {
    // TODO: implement default task assignment
  }
};

} // namespace task_assigner

#endif
