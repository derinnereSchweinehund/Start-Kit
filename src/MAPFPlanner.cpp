#include "SharedEnv.h"
#include "flow.hpp"
#include "heuristics.hpp"
#include "pibt.hpp"
#include <MAPFPlanner.h>
#include <algorithm>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>

struct AstarNode {
  int location;
  int direction;
  int f, g, h;
  AstarNode *parent;
  int t = 0;
  bool closed = false;
  AstarNode(int _location, int _direction, int _g, int _h, AstarNode *_parent)
      : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h),
        parent(_parent) {}
  AstarNode(int _location, int _direction, int _g, int _h, int _t,
            AstarNode *_parent)
      : location(_location), direction(_direction), f(_g + _h), g(_g), h(_h),
        t(_t), parent(_parent) {}
};

struct cmp {
  bool operator()(AstarNode *a, AstarNode *b) {
    if (a->f == b->f)
      return a->g <= b->g;
    else
      return a->f > b->f;
  }
};

void planner::MAPFPlanner::initialize(const SharedEnvironment *initial_state,
                                      double preprocess_time_limit) {
  int map_size = grid_->rows * grid_->cols;
  heuristics.resize(map_size);
  assert(initial_state->num_of_agents_ != 0);
  p.resize(initial_state->num_of_agents_);
  decision.resize(map_size, -1);
  prev_states.resize(initial_state->num_of_agents_);
  next_states.resize(initial_state->num_of_agents_);
  tasks.resize(initial_state->num_of_agents_);
  decided.resize(initial_state->num_of_agents_, DCR({-1, DONE::DONE}));
  occupied.resize(map_size, false);
  checked.resize(initial_state->num_of_agents_, false);
  ids.resize(initial_state->num_of_agents_);
  for (int i = 0; i < ids.size(); i++) {
    ids.at(i) = i;
  }

  std::shuffle(ids.begin(), ids.end(), std::mt19937(std::random_device()()));
  for (int i = 0; i < ids.size(); i++) {
    p[ids.at(i)] = ((double)(ids.size() - i)) / ((double)(ids.size() + 1));
  }
  p_copy = p;

  // if map name inlucde warehouse or sortation, then we need to build traffic
  //if (initial_state->map_name.find("warehouse") != std::string::npos ||
      //initial_state->map_name.find("sortation") != std::string::npos)
    //traffic_control = true;

  //if (traffic_control) {
    //build_traffic(initial_state, traffic);
    //build_traffic_endpoint_warehouse(initial_state, traffic);
  //} else
    //traffic.resize(initial_state->map.size(), -1);
}

vector<Action> planner::MAPFPlanner::query(std::vector<int> start_locations,
                                           std::vector<int> goal_locations,
                                           double time_limit){};

// return next states for all agents
void planner::MAPFPlanner::plan(int time_limit, vector<Action> &actions) {

  prev_decision.clear();
  prev_decision.resize(env->map.size(), -1);
  occupied.clear();
  occupied.resize(env->map.size(), false);

  int ch_count = 0;

  for (int i = 0; i < env->num_of_agents_; i++) {

    if (ch_count < 100)
      for (int j = 0; j < env->goal_locations.at(i).size(); j++) {
        int goal_loc = env->goal_locations.at(i).at(j).first;
        if (heuristics.at(goal_loc).empty()) {
          if (traffic_control)
            compute_heuristics(env, heuristics[goal_loc], traffic, goal_loc);
          else
            compute_heuristics(env, heuristics[goal_loc], goal_loc);
          ch_count++;
        }
      }
    assert(env->goal_locations.at(i).size() > 0);
    tasks.at(i) = env->goal_locations.at(i).front().first;
    assert(env->curr_states.at(i).location >= 0);
    prev_states.at(i) = env->curr_states.at(i);
    next_states.at(i) = State();
    prev_decision[env->curr_states.at(i).location] = i;
    if (decided.at(i).loc == -1) {
      decided.at(i).loc = env->curr_states.at(i).location;
      assert(decided.at(i).state == DONE::DONE);
    }
    if (prev_states.at(i).location == decided.at(i).loc) {
      decided.at(i).state = DONE::DONE;
    }
    if (decided.at(i).state == DONE::NOT_DONE) {
      occupied.at(decided.at(i).loc) = true;
      occupied.at(prev_states.at(i).location) = true;
    }

    if (prev_states.at(i).location == tasks.at(i))
      p.at(i) = p_copy.at(i);
    else
      p.at(i) = p.at(i) + 1;
  }

  std::sort(ids.begin(), ids.end(),
            [&](int a, int b) { return p.at(a) > p.at(b); });

  for (int i : ids) {
    if (decided.at(i).state == DONE::NOT_DONE) {
      continue;
    }
    if (next_states.at(i).location == -1) {
      assert(prev_states.at(i).location >= 0 &&
             prev_states.at(i).location < env->map.size());
      constraintPIBT(i, -1, prev_states, next_states, env, prev_decision,
                     decision, tasks, heuristics, occupied, traffic);
    }
  }

  actions.resize(env->num_of_agents_);
  for (int id : ids) {

    if (next_states.at(id).location != -1)
      decision.at(next_states.at(id).location) = -1;

    assert((next_states.at(id).location >= 0 &&
            decided.at(id).state == DONE::DONE) ||
           (next_states.at(id).location == -1 &&
            decided.at(id).state == DONE::NOT_DONE));

    if (next_states.at(id).location >= 0) {
      decided.at(id) = DCR({next_states.at(id).location, DONE::NOT_DONE});
    }

    actions.at(id) = getAction(prev_states.at(id), decided.at(id).loc, env);
    checked.at(id) = false;
#ifndef NDEBUG
    std::cout << id << ":" << actions.at(id) << ";" << std::endl;
#endif
  }

  for (int id = 0; id < env->num_of_agents_; id++) {
    if (!checked.at(id) && actions.at(id) == Action::FW) {
      moveCheck(id, checked, decided, actions, prev_decision);
    }
  }

#ifndef NDEBUG
  for (auto d : decision) {
    assert(d == -1);
  }
#endif

  prev_states = next_states;
  return;
}

int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {
  int loc1_x = loc1 / env->cols;
  int loc1_y = loc1 % env->cols;
  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2) {
  int loc_x = loc / env->cols;
  int loc_y = loc % env->cols;

  if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
    return false;

  int loc2_x = loc2 / env->cols;
  int loc2_y = loc2 % env->cols;
  if (abs(loc_x - loc2_x) + abs(loc_y - loc2_y) > 1)
    return false;
  return true;
}

list<pair<int, int>> MAPFPlanner::getNeighbors(int location, int direction) {
  list<pair<int, int>> neighbors;
  // forward
  int candidates[4] = {location + 1, location + env->cols, location - 1,
                       location - env->cols};
  int forward = candidates[direction];
  int new_direction = direction;
  if (forward >= 0 && forward < env->map.size() &&
      validateMove(forward, location))
    neighbors.emplace_back(make_pair(forward, new_direction));
  // turn left
  new_direction = direction - 1;
  if (new_direction == -1)
    new_direction = 3;
  neighbors.emplace_back(make_pair(location, new_direction));
  // turn right
  new_direction = direction + 1;
  if (new_direction == 4)
    new_direction = 0;
  neighbors.emplace_back(make_pair(location, new_direction));
  neighbors.emplace_back(make_pair(location, direction)); // wait
  return neighbors;
}
