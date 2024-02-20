#include "ActionModel.h"
#include "ActionSimulator.hpp"
#include "FreeState.h"
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/flat_stream.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/http/dynamic_body.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/beast/http/read.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/beast/http/verb.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
using json = nlohmann::json;
using tcp = boost::beast::net::ip::tcp;

class TurtlebotSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  TurtlebotSimulator(ActionModelWithRotate &model, SharedEnvironment *env)
      : stream_(ioc_), resolver_(ioc_), ActionSimulator(model, env) {}

  vector<Status> simulate_action(vector<Action> &next_actions) override;

  bool validate_safe(const vector<Action> &next_actions) override;

private:
  const string hostname;
  const string port;
  boost::asio::io_service ioc_;
  beast::tcp_stream stream_;
  tcp::resolver resolver_;

  const string post_path = "/extend_path";
  const string get_path = "/get_status";

  inline float location_to_x(int location) {
    return static_cast<float>(location % env->cols);
  };

  inline float location_to_y(int location) {
    return static_cast<float>(location / env->cols);
  };

  inline int xy_to_location(int x, int y) { return y * env->cols + x; }

  FreeState transform_state(const State &place) {
    // std::cout << place.location << " " << location_to_x(place.location) << "
    // "
    //           << location_to_y(place.location) << " " << cols_ << std::endl;
    return FreeState{.x = location_to_x(place.location),
                     .y = location_to_y(place.location),
                     .theta = static_cast<float>(place.orientation * 90),
                     .timestep = place.timestep};
  }
  // I could create a whole http client component class

  json stateToJSON(int agent_id, FreeState &state) {
    json obj = json::object();

    obj["x"] = state.x;
    obj["y"] = -state.y;
    obj["timestep"] = state.timestep;
    obj["theta"] = state.theta;
    obj["agent_id"] = agent_id;

    return obj;
  }

  json construct_payload(const vector<State> &next_states) {
    json payload = json::object();
    json plans = json::array();

    for (int i = 0; i < env->num_of_agents; i++) {
      FreeState next = transform_state(next_states.at(i));
      plans.push_back(stateToJSON(i, next));
    }

    payload["plans"] = plans;
    return payload;
  }

  void connect_to_server() {
    stream_.close();
    auto const results = resolver_.resolve(hostname, port);
    stream_.connect(results);
  }

  template<class body>
  void fill_request(http::request<body> req, const http::verb verb, const string path, const json payload) {
    req.method(verb);
    req.target(path);
    //   req,set(http::field::content_type, "application/json");
    req.body() = payload.dump();
    req.prepare_payload();
  }

  template<class req_body, class resp_body>
  int send_request(http::request<req_body> request, http::response<resp_body> response) {
    connect_to_server();
    http::write(stream_, request);

    beast::flat_buffer buffer;
    http::read(stream_, buffer, response);
    beast::error_code ec;
    (void)stream_.socket().wait(boost::asio::ip::tcp::socket::wait_write, ec);
    stream_.socket().close();
 
    return response.base().result_int();
  }

  http::response<http::dynamic_body>
  send_next_states(const vector<State> &next_states) {

    json payload = construct_payload(next_states);

    http::request<http::string_body> req;
    fill_request(req, http::verb::post, post_path, payload);

    http::response<http::dynamic_body> res;
    int response_code = send_request(req, res);

    if (response_code != 200) { 
      std::cout << "Unsuccessful POST" << std::endl;
    }
    return res;
  }
};
