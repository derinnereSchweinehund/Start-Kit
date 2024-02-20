#include "SharedEnv.h"
#include "planner_wrapper.h"

namespace execution_policy {
template <class P> class ExecutionPolicy {

public:
  ExecutionPolicy(P *planner) : planner_(planner_) {}

  std::vector<Action> &get_actions(const SharedEnvironment &env) {

    return planner_->query(env.curr_states, env.goal_locations);
  }

private:
  P *const planner_;
};
} // namespace execution_policy
