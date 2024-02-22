#ifndef TASKS_H
#define TASKS_H

namespace tasks {
enum TaskStatus { UNASSIGNED = 0, ASSIGNED = 1, COMPLETED = 2 };

struct Task {
  int task_id;
  int location;
  TaskStatus status;
  int agent_assigned;

  Task(int task_id, int location)
      : task_id(task_id), location(location), status(TaskStatus::COMPLETED),
        agent_assigned(-1){};
  Task(int task_id, int location, int t_assigned, int agent_assigned)
      : task_id(task_id), location(location){};
};

} // namespace tasks
  //
#endif // TASKS_H
