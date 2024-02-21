#include "SharedEnv.h"
#include "planner_wrapper.h"

namespace execution_policy {

template <class P> class ExecutionPolicy {

public:
  ExecutionPolicy(P *planner) : planner_(planner_) {}

  std::vector<Action> &get_actions(const SharedEnvironment &state) {

    return planner_->query(state.current_states_, state.goal_locations);
  }

private:
  P *const planner_;
};

typedef planner::wrapper<planner::MAPFPlannerWrapper> MAPFExecutionPolicy;
} // namespace execution_policy
