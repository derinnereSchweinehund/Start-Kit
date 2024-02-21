#include "SharedEnv.h"
#include "Tasks.h"

namespace task_generator {

class TaskGenerator {
public:
  TaskGenerator(std::vector<Task> tasks) : all_tasks_(tasks) {}

  bool update_task(SharedEnvironment *state) {
    // TODO: mutate current state
    // - check current positions and tick off completed task.
    // - check reveal times of hidden tasks.
  }

private:
  std::vector<Task> all_tasks_;
};
} // namespace task_generator
