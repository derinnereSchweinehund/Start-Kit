#include "turtlebot/TurtlebotSimulator.hpp"
#include "nlohmann/json.hpp"
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http/dynamic_body.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/fields.hpp>
#include <vector>

using json = nlohmann::json;
namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http;   // from <boost/beast/http.hpp>
namespace net = boost::asio;    // from <boost/asio.hpp>
using tcp = net::ip::tcp;       // from <boost/asio/ip/tcp.hpp>


inline bool
TurtlebotSimulator::validate_safe(const vector<Action> &next_actions) {
  return validate_unfailing(next_actions, env, model);
}


vector<Status>
TurtlebotSimulator::simulate_action(vector<Action> &next_actions) {
  if (!validate_safe(next_actions)) {
    return vector<Status>(env->num_of_agents, Status::FAILED);
  }



  vector<State> next_states = model.result_states(env->curr_states, next_actions);

  http::response<http::dynamic_body> res = send_next_states(next_states);

  // Figure out what to do when send failed, retry?

  // Understand response to create Status codes
  json agent_results = get_agent_status();

  vector<State> curr_states = parseStates(agent_results["locations"], env->curr_timestep);
  vector<Status> curr_status = parseStatus(agent_results["status"]);
  env->curr_states = curr_states;
  return curr_status;
}
