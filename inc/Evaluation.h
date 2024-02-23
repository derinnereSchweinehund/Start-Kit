#ifndef EVALUATION_H
#define EVALUATION_H
#include "MAPFPlanner.h"

class DummyPlanner : public planner::MAPFPlanner {
private:
  std::vector<std::deque<Action>> agent_plans;

public:
  DummyPlanner(const Grid *const grid) : planner::MAPFPlanner(grid){};
  DummyPlanner(std::string fname, const Grid *const grid)
      : planner::MAPFPlanner(grid) {
    load_plans(fname);
  };
  ~DummyPlanner() {}

  void load_plans(std::string fname);

  virtual std::vector<Action> plan(int time_limit);
};
#endif
