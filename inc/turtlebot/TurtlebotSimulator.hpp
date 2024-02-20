#include "ActionModel.h"
#include "ActionSimulator.hpp"
#include "FreeState.h"
#include "SharedEnv.h"
#include <boost/asio/io_service.hpp>

class TurtlebotSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  TurtlebotSimulator(ActionModelWithRotate &model, SharedEnvironment *env)
      : ActionSimulator(model, env) {}

    vector<Status> simulate_action(vector<Action> &next_actions) override;

    bool validate_safe(const vector<Action> &next_actions) override;

private:
  const string hostname;
  const string port;
  boost::asio::io_service ioc_;
  
  inline float location_to_x(int location) {
    return static_cast<float>(location % env->cols);
  };

  inline float location_to_y(int location) {
    return static_cast<float>(location / env->cols);
  };

  inline int xy_to_location(int x, int y) { return y * env->cols + x; }

  FreeState transform_state(State &place) {
    // std::cout << place.location << " " << location_to_x(place.location) << "
    // "
    //           << location_to_y(place.location) << " " << cols_ << std::endl;
    return FreeState{.x = location_to_x(place.location),
                     .y = location_to_y(place.location),
                     .theta = static_cast<float>(place.orientation * 90),
                     .timestep = place.timestep};
  }

  vector<FreeState> prepare_next_agent_poses(vector<State> &next) {
    std::cout << "Size of next poses: " << std::to_string(next.size())
              << std::endl;
    vector<FreeState> next_agent_poses(next.size());
    for (int i = 0; i < next.size(); i++) {
      next_agent_poses[i] = transform_state(next[i]);
    }
    return next_agent_poses;
  };

  
};
