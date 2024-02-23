// #include "turtlebot/TurtlebotSimulator.hpp"
// #include "SharedEnv.h"
// #include "nlohmann/json.hpp"
// #include <boost/beast/http.hpp>
// #include <boost/beast/http/dynamic_body.hpp>
// #include <boost/beast/http/string_body.hpp>
// #include <vector>

// using json = nlohmann::json;
// namespace beast = boost::beast; // from <boost/beast.hpp>
// namespace http = beast::http;   // from <boost/beast/http.hpp>

// template<class ActionModel>
// inline bool
// TurtlebotSimulator<ActionModel>::validate_safe(const SharedEnvironment &state,
//                                   const vector<Action> &next_actions) {
//   return validate_unfailing(next_actions, state, model);
// }

// template<class ActionModel>
// void TurtlebotSimulator<ActionModel>::simulate_action(SharedEnvironment &state,
//                                          const vector<Action> &next_action) {
//   if (!validate_safe(state, next_action)) {
//     for (Status agent_status : state.current_status_) {
//       agent_status = Status::FAILED;
//     }
//     return;
//   }

//   vector<State> next_states =
//       model.result_states(state.current_states_, next_action);

//   http::response<http::dynamic_body> res = send_next_states(next_states);

//   assert(res.base().result_int() == 200); // Assert successful http

//   // Understand response to create Status codes
//   json agent_results = get_agent_status();

//   state.current_states_ = parseStates(agent_results["locations"],
//                                       state.timestep_, state.num_of_agents_);
//   state.current_status_ =
//       parseStatus(agent_results["status"], state.num_of_agents_);
//   return;
// }
