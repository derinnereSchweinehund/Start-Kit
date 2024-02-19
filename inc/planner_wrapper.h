#include "ActionModel.h"
#include "timer.h"
#include <cstddef>
#include <vector>

struct metrics_t {
  size_t num_queries_;
  double planning_time_nanos_;

  metrics_t() : num_queries_(0), planning_time_nanos_(0) {}
};

template <class P> class PlannerWrapper {
public:
  PlannerWrapper(P *planner) : planner_(planner), metrics_(), timer_() {}

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> &query(const std::vector<size_t> &starts,
                             const std::vector<size_t> &goals,
                             double time_limit = 0.0) {

    metrics_.num_queries_++;
    timer_.start();

    std::vector<Action> &actions = planner_->query(starts, goals);

    metrics_.planning_time_nanos_ += timer_.elapsed_time_nano();

    return actions;
  }

  // Returns the metrics of the planner
  const metrics_t &get_metrics() const { return metrics_; }

private:
  P *const planner_;
  metrics_t metrics_;
  timer timer_;
};
