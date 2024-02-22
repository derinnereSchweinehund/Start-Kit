#include "ActionModel.h"
#include "Logger.h"
#include "SharedEnv.h"
#include "Status.hpp"
#include "Tasks.h"
#include <future>
#include <pthread.h>

namespace base_system {

struct metrics_t {
  std::vector<vector<Action>> actual_movements;
  std::vector<vector<Action>> planner_movements;
  std::vector<Path> paths;
  std::vector<size_t> solution_costs;
  size_t num_of_task_finish = 0;
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

  ~BaseSystem() {
    // safely exit: wait for join the thread then delete planner and exit
    if (started) {
      task_td.join();
    }
    if (planner_ != nullptr) {
      delete planner_;
    }
  };

  void simulate(int simulation_time);

  void savePaths(const string &fileName,
                 int option) const; // option = 0: save actual movement, option
                                    // = 1: save planner movement
  // void saveSimulationIssues(const string &fileName) const;
  void saveResults(const string &fileName, int screen) const;

private:
  Task_Generator *task_generator_;
  Task_Assigner *task_assigner_;
  Execution_Policy *execution_policy_;
  Planner *planner_;
  Simulator *simulator_;
  Logger *logger_ = nullptr;

  SharedEnvironment state_;
  metrics_t metrics_;

  std::future<std::vector<Action>> future;
  std::thread task_td;
  bool started = false;

  std::vector<Path> paths;
  std::vector<std::list<Task>> finished_tasks; // location + finish time

  vector<list<std::tuple<int, int, std::string>>> events;

  // For execution simulation
  // for evaluation
  bool fast_mover_feasible = true;

  void initialize();
  bool planner_initialize();
  void update_tasks();

  void sync_shared_env();
  bool all_tasks_complete();
  void task_book_keeping(vector<Action> &actions);
  void move(vector<Action> &actions);
  bool valid_moves(vector<State> &prev, vector<Action> &next);

  void log_preprocessing(bool succ);
  void log_event_assigned(int agent_id, int task_id, int timestep);
  void log_event_finished(int agent_id, int task_id, int timestep);
};

// class FixedAssignSystem : public BaseSystem {
// public:
// FixedAssignSystem(Grid &grid, string agent_task_filename,
// MAPFPlanner *planner, ActionModelWithRotate *model,
// ActionExecutor *executor)
//: BaseSystem(grid, planner, model,
// new TurtlebotExecutor(grid.rows, grid.cols)) {
// load_agent_tasks(agent_task_filename);
//};

// FixedAssignSystem(Grid &grid, MAPFPlanner *planner,
// std::vector<int> &start_locs,
// std::vector<vector<int>> &tasks,
// ActionModelWithRotate *model)
//: BaseSystem(grid, planner, model,
// new TurtlebotExecutor(grid.rows, grid.cols)) {
// if (start_locs.size() != tasks.size()) {
// std::cerr << "agent num does not match the task assignment" << std::endl;
// exit(1);
//}

// int task_id = 0;
// num_of_agents = start_locs.size();
// starts.resize(num_of_agents);
// task_queue.resize(num_of_agents);
// for (size_t i = 0; i < start_locs.size(); i++) {
// starts[i] = State(start_locs[i], 0, 0);
// for (auto &task_location : tasks[i]) {
// all_tasks.emplace_back(task_id++, task_location, 0, (int)i);
// task_queue[i].emplace_back(
// all_tasks.back().task_id, all_tasks.back().location,
// all_tasks.back().t_assigned, all_tasks.back().agent_assigned);
//}
// task_queue[i] = deque<int>(tasks[i].begin(), tasks[i].end());
//}
//};

//~FixedAssignSystem(){};

// bool load_agent_tasks(string fname);

// private:
// vector<deque<Task>> task_queue;

// void update_tasks();
//};

// class TaskAssignSystem : public BaseSystem {
// public:
// TaskAssignSystem(Grid &grid, MAPFPlanner *planner,
// std::vector<int> &start_locs, std::vector<int> &tasks,
// ActionModelWithRotate *model)
//: BaseSystem(grid, planner, model,
// new TurtlebotExecutor(grid.rows, grid.cols)) {
// int task_id = 0;
// for (auto &task_location : tasks) {
// all_tasks.emplace_back(task_id++, task_location);
// task_queue.emplace_back(all_tasks.back().task_id,
// all_tasks.back().location);
//// task_queue.emplace_back(task_id++, task_location);
//}
// num_of_agents = start_locs.size();
// starts.resize(num_of_agents);
// for (size_t i = 0; i < start_locs.size(); i++) {
// starts[i] = State(start_locs[i], 0, 0);
//}
//};

//~TaskAssignSystem(){};

// private:
// deque<Task> task_queue;

// void update_tasks();
//};

// class InfAssignSystem : public BaseSystem {
// public:
// InfAssignSystem(Grid &grid, MAPFPlanner *planner,
// std::vector<int> &start_locs, std::vector<int> &tasks,
// ActionModelWithRotate *model)
//: tasks(tasks), BaseSystem(grid, planner, model,
// new TurtlebotExecutor(grid.rows, grid.cols)) {
// num_of_agents = start_locs.size();
// starts.resize(num_of_agents);
// task_counter.resize(num_of_agents, 0);
// tasks_size = tasks.size();

// for (size_t i = 0; i < start_locs.size(); i++) {
//
// if (grid.map[start_locs[i]] == 1) {
// cout << "error: agent " << i << "'s start location is an obstacle("
//<< start_locs[i] << ")" << endl;
// exit(0);
//}
// starts[i] = State(start_locs[i], 0, 0);
//}
//};

//~InfAssignSystem(){};

// private:
// std::vector<int> &tasks;
// std::vector<int> task_counter;
// int tasks_size;
// int task_id = 0;

// void update_tasks();
//};
//

} // namespace base_system
