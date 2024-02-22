#include "CompetitionSystem.h"

using json = nlohmann::ordered_json;

namespace base_system {

template <class Task_Generator, class Task_Assigner, class Execution_Policy,
          class Planner, class Simulator>
void BaseSystem<Task_Generator, Task_Assigner, Execution_Policy, Planner,
                Simulator>::simulate(SharedEnvironment &state,
                                     int simulation_time) {

  while (task_generator_->update_tasks(state)) {
    task_assigner_->assign_tasks(state);
    // TODO: validate user defined functions
    execution_policy_->get_actions(state);
    // TODO: validate user defined functions
    simulator_->simulate_actions(state);
  }
}

//template <class Task_Generator, class Task_Assigner, class Execution_Policy,
          //class Planner, class Simulator>
//void BaseSystem<Task_Generator, Task_Assigner, Execution_Policy, Planner,
                //Simulator>::save_paths(const string &fileName,
                                       //int option) const {
  //std::ofstream output;
  //output.open(fileName, std::ios::out);
  //for (int i = 0; i < state_.num_of_agents_; i++) {
    //output << "Agent " << i << ": ";
    //if (option == 0) {
      //bool first = true;
      //for (const auto t : metrics_.actual_movements[i]) {
        //if (!first) {
          //output << ",";
        //} else {
          //first = false;
        //}
        //output << t;
      //}
    //} else if (option == 1) {
      //bool first = true;
      //for (const auto t : metrics_.planner_movements[i]) {
        //if (!first) {
          //output << ",";
        //} else {
          //first = false;
        //}
        //output << t;
      //}
    //} else if (option == 3) {
      //bool first = true;
      //for (const auto t : metrics_.paths[i]) {
        //if (!first) {
          //output << ",";
        //} else {
          //first = false;
        //}
        //output << t;
      //}
    //}
    //output << endl;
  //}
  //output.close();
//}

//template <class Task_Generator, class Task_Assigner, class Execution_Policy,
          //class Planner, class Simulator>
//void BaseSystem<Task_Generator, Task_Assigner, Execution_Policy, Planner,
                //Simulator>::save_results(const string &fileName,
                                         //int screen) const {
  //json js;
  //// Save action model
  //js["actionModel"] = "MAPF_T";

  //js["AllValid"] = feasible;

  //js["teamSize"] = state_.num_of_agents_;

  //// Save start locations[x,y,orientation]
  //if (screen <= 2) {
    //json start = json::array();
    //for (int i = 0; i < state_.num_of_agents_; i++) {
      //json s = json::array();
      //s.push_back(starts[i].location / map.cols);
      //s.push_back(starts[i].location % map.cols);
      //switch (starts[i].orientation) {
      //case 0:
        //s.push_back("E");
        //break;
      //case 1:
        //s.push_back("S");
      //case 2:
        //s.push_back("W");
        //break;
      //case 3:
        //s.push_back("N");
        //break;
      //}
      //start.push_back(s);
    //}
    //js["start"] = start;
  //}

  //js["numTaskFinished"] = num_of_task_finish;
  //int sum_of_cost = 0;
  //int makespan = 0;
  //if (num_of_agents > 0) {
    //sum_of_cost = solution_costs[0];
    //makespan = solution_costs[0];
    //for (int a = 1; a < num_of_agents; a++) {
      //sum_of_cost += solution_costs[a];
      //if (solution_costs[a] > makespan) {
        //makespan = solution_costs[a];
      //}
    //}
  //}
  //js["sumOfCost"] = sum_of_cost;
  //js["makespan"] = makespan;

  //if (screen <= 2) {
    //// Save actual paths
    //json apaths = json::array();
    //for (int i = 0; i < num_of_agents; i++) {
      //std::string path;
      //bool first = true;
      //for (const auto action : actual_movements[i]) {
        //if (!first) {
          //path += ",";
        //} else {
          //first = false;
        //}

        //if (action == Action::FW) {
          //path += "F";
        //} else if (action == Action::CR) {
          //path += "R";
        //} else if (action == Action::CCR) {
          //path += "C";
        //} else if (action == Action::NA) {
          //path += "T";
        //} else {
          //path += "W";
        //}
      //}
      //apaths.push_back(path);
    //}
    //js["actualPaths"] = apaths;
  //}

  //if (screen <= 1) {
    //// planned paths
    //json ppaths = json::array();
    //for (int i = 0; i < num_of_agents; i++) {
      //std::string path;
      //bool first = true;
      //for (const auto action : planner_movements[i]) {
        //if (!first) {
          //path += ",";
        //} else {
          //first = false;
        //}

        //if (action == Action::FW) {
          //path += "F";
        //} else if (action == Action::CR) {
          //path += "R";
        //} else if (action == Action::CCR) {
          //path += "C";
        //} else if (action == Action::NA) {
          //path += "T";
        //} else {
          //path += "W";
        //}
      //}
      //ppaths.push_back(path);
    //}
    //js["plannerPaths"] = ppaths;

    //json planning_times = json::array();
    //for (double time : planner_times)
      //planning_times.push_back(time);
    //js["plannerTimes"] = planning_times;

    //// Save errors
    //json errors = json::array();
    //for (auto error : model->errors) {
      //std::string error_msg;
      //int agent1;
      //int agent2;
      //int timestep;
      //std::tie(error_msg, agent1, agent2, timestep) = error;
      //json e = json::array();
      //e.push_back(agent1);
      //e.push_back(agent2);
      //e.push_back(timestep);
      //e.push_back(error_msg);
      //errors.push_back(e);
    //}
    //js["errors"] = errors;

    //// Save events
    //json events_json = json::array();
    //for (int i = 0; i < num_of_agents; i++) {
      //json event = json::array();
      //for (auto e : events[i]) {
        //json ev = json::array();
        //std::string event_msg;
        //int task_id;
        //int timestep;
        //std::tie(task_id, timestep, event_msg) = e;
        //ev.push_back(task_id);
        //ev.push_back(timestep);
        //ev.push_back(event_msg);
        //event.push_back(ev);
      //}
      //events_json.push_back(event);
    //}
    //js["events"] = events_json;

    //// Save all tasks
    //json tasks = json::array();
    //for (auto t : all_tasks) {
      //json task = json::array();
      //task.push_back(t.task_id);
      //task.push_back(t.location / map.cols);
      //task.push_back(t.location % map.cols);
      //tasks.push_back(task);
    //}
    //js["tasks"] = tasks;
  //}

  //std::ofstream f(fileName, std::ios_base::trunc | std::ios_base::out);
  //f << std::setw(4) << js;
//}

} // namespace base_system
