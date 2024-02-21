#include "CompetitionSystem.h"
#include "nlohmann/json.hpp"
#include <Logger.h>
#include <boost/tokenizer.hpp>
#include <functional>
#include <unistd.h>

using json = nlohmann::ordered_json;

bool BaseSystem::all_tasks_complete() {
  for (auto &t : assigned_tasks) {
    if (!t.empty()) {
      return false;
    }
  }
  return true;
}

void BaseSystem::task_book_keeping(vector<Action> &actions) {

  list<Task> finished_tasks_this_timestep; // <agent_id, task_id, timestep>

  for (int k = 0; k < num_of_agents; k++) {
    if (!assigned_tasks[k].empty() &&
        curr_states[k].location == assigned_tasks[k].front().location) {
      Task task = assigned_tasks[k].front();
      assigned_tasks[k].pop_front();
      task.t_completed = timestep;
      finished_tasks_this_timestep.push_back(task);
      events[k].push_back(make_tuple(task.task_id, timestep, "finished"));
      // log_event_finished(k, task.task_id, timestep);
    }
    paths[k].push_back(curr_states[k]);
    // Use this paths[] to verify for 1-robustness in MAPF-DP, generate a
    // warning for now? Feels like I should add a MAPF-DP flag?

    actual_movements[k].push_back(actions[k]);
  }
  // update tasks
  //
  for (auto task : finished_tasks_this_timestep) {
    finished_tasks[task.agent_assigned].emplace_back(task);
    num_of_task_finish++;
  }

  // TODO: this should be dependency injection
  update_tasks();
}

void BaseSystem::log_preprocessing(bool succ) {
  if (logger == nullptr)
    return;
  if (succ) {
    logger->log_info("Preprocessing success", timestep);
  } else {
    logger->log_fatal("Preprocessing timeout", timestep);
  }
  logger->flush();
}

void BaseSystem::log_event_assigned(int agent_id, int task_id, int timestep) {
  logger->log_info("Task " + std::to_string(task_id) +
                       " is assigned to agent " + std::to_string(agent_id),
                   timestep);
}

void BaseSystem::log_event_finished(int agent_id, int task_id, int timestep) {
  logger->log_info("Agent " + std::to_string(agent_id) + " finishes task " +
                       std::to_string(task_id),
                   timestep);
}

void BaseSystem::simulate(int simulation_time) {

  while (true) {
    task_generator_->update_tasks(state_);
    task_assigner_->assign_tasks(state_);
    execution_policy_->get_actions(state_);
    simulator_->simulate_actions(state_);
  }
}

void BaseSystem::savePaths(const string &fileName, int option) const {
  std::ofstream output;
  output.open(fileName, std::ios::out);
  for (int i = 0; i < num_of_agents; i++) {
    output << "Agent " << i << ": ";
    if (option == 0) {
      bool first = true;
      for (const auto t : actual_movements[i]) {
        if (!first) {
          output << ",";
        } else {
          first = false;
        }
        output << t;
      }
    } else if (option == 1) {
      bool first = true;
      for (const auto t : planner_movements[i]) {
        if (!first) {
          output << ",";
        } else {
          first = false;
        }
        output << t;
      }
    } else if (option == 3) {
      bool first = true;
      for (const auto t : paths[i]) {
        if (!first) {
          output << ",";
        } else {
          first = false;
        }
        output << t;
      }
    }
    output << endl;
  }
  output.close();
}

void BaseSystem::saveResults(const string &fileName, int screen) const {
  json js;
  // Save action model
  js["actionModel"] = "MAPF_T";

  std::string feasible = fast_mover_feasible ? "Yes" : "No";
  js["AllValid"] = feasible;

  js["teamSize"] = num_of_agents;

  // Save start locations[x,y,orientation]
  if (screen <= 2) {
    json start = json::array();
    for (int i = 0; i < num_of_agents; i++) {
      json s = json::array();
      s.push_back(starts[i].location / map.cols);
      s.push_back(starts[i].location % map.cols);
      switch (starts[i].orientation) {
      case 0:
        s.push_back("E");
        break;
      case 1:
        s.push_back("S");
      case 2:
        s.push_back("W");
        break;
      case 3:
        s.push_back("N");
        break;
      }
      start.push_back(s);
    }
    js["start"] = start;
  }

  js["numTaskFinished"] = num_of_task_finish;
  int sum_of_cost = 0;
  int makespan = 0;
  if (num_of_agents > 0) {
    sum_of_cost = solution_costs[0];
    makespan = solution_costs[0];
    for (int a = 1; a < num_of_agents; a++) {
      sum_of_cost += solution_costs[a];
      if (solution_costs[a] > makespan) {
        makespan = solution_costs[a];
      }
    }
  }
  js["sumOfCost"] = sum_of_cost;
  js["makespan"] = makespan;

  if (screen <= 2) {
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++) {
      std::string path;
      bool first = true;
      for (const auto action : actual_movements[i]) {
        if (!first) {
          path += ",";
        } else {
          first = false;
        }

        if (action == Action::FW) {
          path += "F";
        } else if (action == Action::CR) {
          path += "R";
        } else if (action == Action::CCR) {
          path += "C";
        } else if (action == Action::NA) {
          path += "T";
        } else {
          path += "W";
        }
      }
      apaths.push_back(path);
    }
    js["actualPaths"] = apaths;
  }

  if (screen <= 1) {
    // planned paths
    json ppaths = json::array();
    for (int i = 0; i < num_of_agents; i++) {
      std::string path;
      bool first = true;
      for (const auto action : planner_movements[i]) {
        if (!first) {
          path += ",";
        } else {
          first = false;
        }

        if (action == Action::FW) {
          path += "F";
        } else if (action == Action::CR) {
          path += "R";
        } else if (action == Action::CCR) {
          path += "C";
        } else if (action == Action::NA) {
          path += "T";
        } else {
          path += "W";
        }
      }
      ppaths.push_back(path);
    }
    js["plannerPaths"] = ppaths;

    json planning_times = json::array();
    for (double time : planner_times)
      planning_times.push_back(time);
    js["plannerTimes"] = planning_times;

    // Save errors
    json errors = json::array();
    for (auto error : model->errors) {
      std::string error_msg;
      int agent1;
      int agent2;
      int timestep;
      std::tie(error_msg, agent1, agent2, timestep) = error;
      json e = json::array();
      e.push_back(agent1);
      e.push_back(agent2);
      e.push_back(timestep);
      e.push_back(error_msg);
      errors.push_back(e);
    }
    js["errors"] = errors;

    // Save events
    json events_json = json::array();
    for (int i = 0; i < num_of_agents; i++) {
      json event = json::array();
      for (auto e : events[i]) {
        json ev = json::array();
        std::string event_msg;
        int task_id;
        int timestep;
        std::tie(task_id, timestep, event_msg) = e;
        ev.push_back(task_id);
        ev.push_back(timestep);
        ev.push_back(event_msg);
        event.push_back(ev);
      }
      events_json.push_back(event);
    }
    js["events"] = events_json;

    // Save all tasks
    json tasks = json::array();
    for (auto t : all_tasks) {
      json task = json::array();
      task.push_back(t.task_id);
      task.push_back(t.location / map.cols);
      task.push_back(t.location % map.cols);
      tasks.push_back(task);
    }
    js["tasks"] = tasks;
  }

  std::ofstream f(fileName, std::ios_base::trunc | std::ios_base::out);
  f << std::setw(4) << js;
}

bool FixedAssignSystem::load_agent_tasks(string fname) {
  string line;
  std::ifstream myfile(fname.c_str());
  if (!myfile.is_open())
    return false;

  getline(myfile, line);
  while (!myfile.eof() && line[0] == '#') {
    getline(myfile, line);
  }

  boost::char_separator<char> sep(",");
  boost::tokenizer<boost::char_separator<char>> tok(line, sep);
  boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

  num_of_agents = atoi((*beg).c_str());
  int task_id = 0;
  // My benchmark
  if (num_of_agents == 0) {
    // issue_logs.push_back("Load file failed");
    std::cerr << "The number of agents should be larger than 0" << endl;
    exit(-1);
  }
  starts.resize(num_of_agents);
  task_queue.resize(num_of_agents);

  for (int i = 0; i < num_of_agents; i++) {

    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#')
      getline(myfile, line);

    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    // read start [row,col] for agent i
    int num_landmarks = atoi((*beg).c_str());
    beg++;
    auto loc = atoi((*beg).c_str());
    // agent_start_locations[i] = {loc, 0};
    starts[i] = State(loc, 0, 0);
    beg++;
    for (int j = 0; j < num_landmarks; j++, beg++) {
      auto loc = atoi((*beg).c_str());
      task_queue[i].emplace_back(task_id++, loc, 0, i);
    }
  }
  myfile.close();

  return true;
}
