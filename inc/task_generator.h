#include "Tasks.h"

namespace task_generator {

class TaskGenerator {
public:
  void update_task(State *state) {
    // TODO: mutate current state
    // - check current positions and tick off completed task.
    // - check reveal times of hidden tasks.
  }

private:
  list<Task> all_tasks_;
};
} // namespace task_generator
