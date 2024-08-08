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

  planner_metrics_t() : num_queries_(0), planning_time_nanos_(0) {}
};

template <class P> class wrapper {
public:
  wrapper(P *planner, Logger *logger)
      : planner_(planner), metrics_(), timer_(), logger_(logger) {
    auto planner_query_fn = [=](const std::vector<State>& a, const std::vector<std::deque<tasks::Task>>& b, double c) {
      //std::cout << "FUTURE" << planner << std::flush;
      //return planner->query(a, b, c);
      return std::vector<Action>(a.size(), Action::W);
      };
    query_ = std::packaged_task<std::vector<Action>(const std::vector<State>&,
                                         const std::vector<std::deque<tasks::Task>>&, double)>(planner_query_fn);
    //query_(std::bind(&P::query, planner, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    //query_ = std::packaged_task<std::vector<Action>(P::*(const std::vector<int>&, const std::vector<int>&, double))>(
        //std::bind(&P::query, &planner_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    //result_ = query_.get_future();
  }

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> query(const std::vector<State> &starts,
                            const std::vector<std::deque<tasks::Task>> &goals,
                            double time_limit = 0.0) {

    metrics_.num_queries_++;
    
    timer_.start();
    std::vector<Action> actions{};
    try {
      result_ = query_.get_future();
      std::thread task(std::move(query_), starts, goals, time_limit); 
    if (result_.wait_for(std::chrono::duration<double>(time_limit)) ==
        std::future_status::ready) {
      std::cout << "PASSED" << std::flush;
      task.join();
      //std::vector<Action> 
      actions = result_.get();
    } else {
      logger_->log_info("planner timeout");
    }

    metrics_.planning_time_nanos_ += timer_.elapsed_time_nano();
    } catch (const std::exception& e) {
      std::cout << '\n' << e.what() << std::flush;
    }
    return actions;
  }

  // Returns the metrics of the planner
  const planner_metrics_t &get_metrics() const { return metrics_; }

private:
  P *const planner_;
  planner_metrics_t metrics_;
  Timer timer_;
  Logger *const logger_;

  //std::packaged_task<std::vector<Action>(P::*(const std::vector<int>&,
                                         //const std::vector<int>&, double))>
  std::packaged_task<std::vector<Action>(const std::vector<State>&, const std::vector<std::deque<tasks::Task>>&, double)> query_;
  std::future<std::vector<Action>> result_;
};

typedef wrapper<MAPFPlanner> MAPFPlannerWrapper;

} // namespace planner
#endif // PLANNER_WRAPPER_H
