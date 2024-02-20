#include "Tasks.h"

namespace task_assigner {
template <class T, class S> class TaskAssignmentSystem {
public:
  TaskAssignmentSystem(T *task_assigner) : task_assigner_(task_assigner) {}

  std::vector<Task> assign_tasks(S state) {
    task_assigner_->assign_tasks(state);
  }

private:
  T *task_assigner_;
};

} // namespace task_assigner
