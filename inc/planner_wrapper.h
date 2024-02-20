#include "ActionModel.h"
#include "timer.h"
#include <cstddef>
#include <future>
#include <vector>

// TODO: add timeout and preporcess time limit

// int preprocess_time_limit = 10;
// int plan_time_limit = 3;

namespace planner {

struct metrics_t {
  size_t num_queries_;
  double planning_time_nanos_;

  metrics_t() : num_queries_(0), planning_time_nanos_(0) {}
};

template <class P> class wrapper {
public:
  wrapper(P *planner, double planning_limit_seconds)
      : planner_(planner), metrics_(), timer_(),
        planning_limit_seconds_(planning_limit_seconds) {

    query_(planner_->query);
    result_ = query_.get_future();
  }

  // Calls the planner's query method and updates the metrics
  // @param start The start states of the agents
  // @param goal The goal states of the agents
  // @return The next action for each agent
  std::vector<Action> query(const std::vector<size_t> &starts,
                            const std::vector<size_t> &goals,
                            double time_limit = 0.0) {

    metrics_.num_queries_++;
    timer_.start();

    std::thread task(std::move(query_), starts, goals, time_limit);

    std::vector<Action> actions{};
    if (result_.wait_for(std::chrono::duration<double>(time_limit)) ==
        std::future_status::ready) {
      task.join();
      std::vector<Action> actions = result_.get();
    }

    metrics_.planning_time_nanos_ += timer_.elapsed_time_nano();
    return actions;
  }

  // Returns the metrics of the planner
  const metrics_t &get_metrics() const { return metrics_; }

private:
  P *const planner_;
  metrics_t metrics_;
  timer timer_;
  double planning_limit_seconds_;

  // async

  std::packaged_task<std::vector<Action>(const std::vector<size_t>,
                                         const std::vector<size_t>, double)>
      query_;
  std::future<std::vector<Action>> result_;
};

} // namespace planner
