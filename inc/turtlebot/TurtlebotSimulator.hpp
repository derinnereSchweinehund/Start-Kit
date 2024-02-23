#pragma once
#include "ActionModel.h"
#include "ActionSimulator.hpp"
#include "FreeState.h"
#include "Grid.h"
#include "SharedEnv.h"
#include "States.h"
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
#include <sys/socket.h>
#include <unistd.h>

namespace beast = boost::beast;
namespace http = beast::http;
using json = nlohmann::json;
using tcp = boost::beast::net::ip::tcp;

class TurtlebotSimulator : ActionSimulator<ActionModelWithRotate> {
public:
  TurtlebotSimulator(ActionModelWithRotate &model, Grid &grid)
      : stream_(ioc_), resolver_(ioc_), ActionSimulator(model, grid) {}

  void simulate_action(SharedEnvironment &state,
                                 const vector<Action> &next_actions) override;

  bool validate_safe(const SharedEnvironment &state,
                     const vector<Action> &next_actions) override;

private:
  const string hostname;
  const string port;
  boost::asio::io_service ioc_;
  beast::tcp_stream stream_;
  tcp::resolver resolver_;

  const string post_path = "/extend_path";
  const string get_path = "/get_status";

  inline float location_to_x(int location, const Grid &grid) {
    return static_cast<float>(location % grid.cols);
  };

  inline float location_to_y(int location, const Grid &grid) {
    return static_cast<float>(location / grid.cols);
  };

  inline int xy_to_location(int x, int y, int cols) { return y * cols + x; }

  FreeState transform_state(const State &place, const Grid &grid) {
    // std::cout << place.location << " " << location_to_x(place.location) << "
    // "
    //           << location_to_y(place.location) << " " << cols_ << std::endl;
    return FreeState{.x = location_to_x(place.location, grid),
                     .y = location_to_y(place.location, grid),
                     .theta = static_cast<float>(place.orientation * 90),
                     .timestep = place.timestep};
  }
  // I could create a whole http client component class

  json stateToJson(int agent_id, FreeState &state) {
    json obj = json::object();

    obj["x"] = state.x;
    obj["y"] = -state.y;
    obj["timestep"] = state.timestep;
    obj["theta"] = state.theta;
    obj["agent_id"] = agent_id;

    return obj;
  }

  json construct_payload(const vector<State> &next_states, int num_of_agents) {
    json payload = json::object();
    json plans = json::array();

    for (int i = 0; i < num_of_agents; i++) {
      FreeState next = transform_state(next_states.at(i), grid);
      plans.push_back(stateToJson(i, next));
    }

    payload["plans"] = plans;
    return payload;
  }

  void connect_to_server() {
    stream_.close();
    auto const results = resolver_.resolve(hostname, port);
    stream_.connect(results);
  }

  template <class body>
  void fill_request(http::request<body> req, const http::verb verb,
                    const string path, const json payload) {
    req.method(verb);
    req.target(path);
    //   req,set(http::field::content_type, "application/json");
    req.body() = payload.dump();
    req.prepare_payload();
  }

  template <class req_body, class resp_body>
  int send_request(http::request<req_body> request,
                   http::response<resp_body> response) {
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

    json payload = construct_payload(next_states, state->num_of_agents_);

    http::request<http::string_body> req;
    fill_request(req, http::verb::post, post_path, payload);

    http::response<http::dynamic_body> res;
    int response_code = send_request(req, res);

    if (response_code != 200) {
      std::cout << "Unsuccessful POST" << std::endl;
    }
    return res;
  }

  json get_agent_status() {
    http::request<http::string_body> req;
    fill_request(req, http::verb::get, get_path, json::object());
    http::response<http::dynamic_body> res;
    
    // This will attempt for 5 minutes
    for (int i = 0; i < 500; i++) {
      int response_code = send_request(req, res);
      if (response_code != 200) {
        std::cout << "Unsuccessful GET" << std::endl;
        sleep(1);
      } else {
        break;
      } 
    }
    assert(res.base().result_int() == 200); // Assert that the request succeeded

    // data has 2 fields {
    //              "locations" : [{x, y, theta, agent_id}], 
    //              "status" : [SUCEEDED|FAILED|IN-PROGRESS]
    //              }

    return json::parse(beast::buffers_to_string(res.body().data()));
  }

  State jsonToState(json state_json, int timestep) {
    int x = state_json["x"].get<int>();
    int y = abs(state_json["y"].get<int>());
    int theta = state_json["theta"].get<int>();
    int agent_id = state_json["agent_id"].get<int>();

    return State(xy_to_location(x, y, grid.cols), timestep,
                 ((theta + 45) / 90) % 4); // Map 0-360 to 0-3
  }

  vector<State> parseStates(json::array_t json_states, int timestep, int num_of_agents) {
    vector<State> curr_states(num_of_agents);
    for (int i = 0; i < json_states.size(); i++) {
      json agent_state = json_states[i];
      curr_states.at(agent_state["agent_id"]) =
          jsonToState(agent_state, timestep);
    }
    // Guarantee that every agent is given an update???
    return curr_states;
  }

  vector<Status> parseStatus(json::array_t json_status, int num_of_agents) {
    vector<Status> curr_status(num_of_agents, Status::UNKNOWN);
    for (int i = 0; i < json_status.size(); i++) {
      json agent_status = curr_status[i];
      string status_str = agent_status["status"].get<string>();
      for (auto &c : status_str)
        c = std::toupper((unsigned char)c);
      if (status_str == "EXECUTING") {
        curr_status.at(agent_status["agent_id"].get<int>()) = Status::EXECUTING;
      } else if (status_str == "FAILED") {
        curr_status.at(agent_status["agent_id"].get<int>()) = Status::FAILED;
      } else if (status_str == "SUCCEEDED") {
        curr_status.at(agent_status["agent_id"].get<int>()) = Status::SUCCEEDED;
      } else {
        curr_status.at(agent_status["agent_id"].get<int>()) = Status::UNKNOWN;
      }
    }

    return curr_status;
  }
};
