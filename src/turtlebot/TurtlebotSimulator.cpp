#include "turtlebot/TurtlebotSimulator.hpp"
#include "nlohmann/json.hpp"
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/fields.hpp>
#include <vector>

using json = nlohmann::json;
namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http;   // from <boost/beast/http.hpp>
namespace net = boost::asio;    // from <boost/asio.hpp>
using tcp = net::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

json stateToJSON(int agent_id, FreeState &state) {
  json obj = json::object();

  obj["x"] = state.x;
  obj["y"] = -state.y;
  obj["timestep"] = state.timestep;
  obj["theta"] = state.theta;
  obj["agent_id"] = agent_id;

  return obj;
}

inline bool
TurtlebotSimulator::validate_safe(const vector<Action> &next_actions) {
  return validate_unfailing(next_actions, env, model);
}

vector<Status>
TurtlebotSimulator::simulate_action(vector<Action> &next_actions) {
  if (!validate_safe(next_actions)) {
    return vector<Status>(env->num_of_agents, Status::FAILED);
  }

  json payload = json::object();

  json plans = json::array();

  vector<State> next_states = model.result_states(env->curr_states, next_actions);

  for (int i = 0; i < env->num_of_agents; i++) {
    FreeState next = transform_state(next_states.at(i));
    plans.push_back(stateToJSON(i, next));
  }

  payload["plans"] = plans;
  
  
  const string path = "/extend_path";

  beast::tcp_stream stream(ioc_);
  tcp::resolver resolver(ioc_);

  auto const results = resolver.resolve(hostname, port);

  stream.connect(results);

  http::request<http::string_body> req;

  req.method(http::verb::post);
  req.target(path);
//   req,set(http::field::content_type, "application/json");
  req.body() = payload.dump();
  req.prepare_payload();

  http::write(stream, req);

  beast::flat_buffer buffer;

  http::response<http::dynamic_body> res;

  http::read(stream, buffer, res);

  if (res.base().result_int() != 200) { // OK response
    std::cout << "Unsuccessful POST" << std::endl;
  }
  // Gracefully close the socket
  beast::error_code ec;
  (void)stream.socket().wait(boost::asio::ip::tcp::socket::wait_write, ec);
  // stream.socket().close();
  // not_connected happens sometimes
  // so don't bother reporting it.
  //
  if (ec && ec != beast::errc::not_connected){
    throw beast::system_error{ec};
  }
    

  
  // Understand response to create Status codes

  vector<Status> status(env->num_of_agents);



  return status;
}
