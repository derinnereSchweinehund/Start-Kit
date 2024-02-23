#ifndef PLANNER_WRAPPER_H
#define PLANNER_WRAPPER_H

#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "timer.h"
#include <cstddef>
#include <future>
#include <vector>

namespace planner {

struct planner_metrics_t {
  size_t num_queries_;
  double planning_time_nanos_;
  double preprocess_time_nanos_;

  planner_metrics_t() : num_queries_(0), planning_time_nanos_(0) {}
};

template <class P> class wrapper {
public:
  wrapper(P *planner, Logger *logger)
      : planner_(planner), metrics_(), timer_(), logger_(logger) {}

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> query(const std::vector<int> &starts,
                            const std::vector<int> &goals,
                            double time_limit = 0.0) {

    metrics_.num_queries_++;
    timer_.start();

    std::packaged_task<std::vector<Action>(const std::vector<int>,
                                           const std::vector<int>, double)>
        query(std::bind(&P::query, planner_, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3));
    std::future<std::vector<Action>> result = query.get_future();
    std::thread task(std::move(query), starts, goals, time_limit);

    std::vector<Action> actions{};
    if (result.wait_for(std::chrono::duration<double>(time_limit)) ==
        std::future_status::ready) {
      task.join();
      std::vector<Action> actions = result.get();
    } else {
      logger_->log_info("planner timeout");
    }

    metrics_.planning_time_nanos_ += timer_.elapsed_time_nano();
    return actions;
  }

  // Returns the metrics of the planner
  const planner_metrics_t &get_metrics() const { return metrics_; }

  void initialize(const SharedEnvironment *initial_state,
                  double preprocess_time_limit) {
    timer_.start();
    planner_->initialize(initial_state, preprocess_time_limit);
    metrics_.preprocess_time_nanos_ += timer_.elapsed_time_nano();
  }

private:
  P *const planner_;
  planner_metrics_t metrics_;
  Timer timer_;
  Logger *const logger_;
};

typedef wrapper<MAPFPlanner> MAPFPlannerWrapper;

} // namespace planner
#endif // PLANNER_WRAPPER_H
