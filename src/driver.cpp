#include "ActionSimulator.hpp"
#include "CompetitionSystem.h"
#include "MAPFPlanner.h"
#include "execution_policy.h"
#include "nlohmann/json.hpp"
#include "planner_wrapper.h"
#include "task_assigner.h"
#include "task_generator.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <climits>
#include <fstream>
#include <memory>
#include <signal.h>

#ifdef PYTHON
#if PYTHON
#include "pyMAPFPlanner.hpp"
#include <pybind11/embed.h>
#endif
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;

// void sigint_handler(int a) {
// fprintf(stdout, "stop the simulation...\n");
// if (!vm["evaluationMode"].as<bool>()) {
// system.saveResults(vm["output"].as<std::string>(),
// vm["outputScreen"].as<int>());
//}
//_exit(0);
//}
std::string action_to_string(const Action &action) {
    switch (action) {
        case FW: return "FW";
        case CR: return "CR";
        case CCR: return "CCR";
        case W: return "W";
        case NA: return "NA";
        default: return "Unknown";
    }
}

// Function to convert State to a string for output
std::string state_to_string(const State &state) {
    return "Location: " + std::to_string(state.location) +
           ", Timestep: " + std::to_string(state.timestep) +
           ", Orientation: " + std::to_string(state.orientation);
}

// Function to convert Path to a string for output
std::string path_to_string(const Path &path) {
    std::string result;
    for (const auto &state : path) {
        result += state_to_string(state) + "\n";
    }
    return result;
}

// Save results to a file and optionally print to the screen
void save_results(const std::string &fileName, 
                  const base_system::metrics_t &system_metrics, 
                  const planner::planner_metrics_t &planner_metrics, 
                  const task_generator::task_generator_metrics_t &task_generator_metrics) {

    // Open file for writing
    std::ofstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    // Write system metrics
    file << "System Metrics:\n";
    file << "Actual Movements:\n";
    for (const auto& agent_movements : system_metrics.actual_movements) {
        for (const auto& action : agent_movements) {
            file << action_to_string(action) << " ";
        }
        file << "\n";
    }
    std::cout << system_metrics.actual_movements.size() << '\n';

    file << "Planner Movements:\n";
    for (const auto& agent_movements : system_metrics.planner_movements) {
        for (const auto& action : agent_movements) {
            file << action_to_string(action) << " ";
        }
        file << "\n";
    }

    file << "Paths:\n";
    for (const auto& path : system_metrics.paths) {
        file << path_to_string(path);
    }

    file << "Solution Costs:\n";
    for (const auto& cost : system_metrics.solution_costs) {
        file << cost << "\n";
    }

    // Write planner metrics
    file << "\nPlanner Metrics:\n";
    file << "Number of Queries: " << planner_metrics.num_queries_ << "\n";
    file << "Planning Time (ns): " << planner_metrics.planning_time_nanos_ << "\n";

    // Write task generator metrics
    file << "\nTask Generator Metrics:\n";
    file << "Number of Tasks Finished: " << task_generator_metrics.num_of_task_finish << "\n";

    // Close the file
    file.close();
}

int main(int argc, char **argv) {

  // #ifdef PYTHON
  //// std::cout<<"Using Python="<<PYTHON<<std::endl;
  // #if PYTHON
  // pybind11::initialize_interpreter();
  // #endif
  // #endif

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")
      ("inputFolder", po::value<std::string>()->default_value("./"), "inputfolder")
      ("inputFile,i", po::value<std::string>()->required(), "input file name")(
          "output,o", po::value<std::string>()->default_value("./test.json"),
          "output file name")(
          "outputScreen", po::value<int>()->default_value(1),
          "the level of details in the output file, 1--showing all the output, "
          "2--ignore the events and tasks, 3--ignore the events, tasks, "
          "errors, planner times, starts and paths")(
          "evaluationMode", po::value<bool>()->default_value(false),
          "evaluate an existing output file")(
          "simulationTime", po::value<int>()->default_value(5000),
          "run simulation")("fileStoragePath",
                            po::value<std::string>()->default_value(""),
                            "the path to the storage path")(
          "planTimeLimit", po::value<int>()->default_value(INT_MAX),
          "the time limit for planner in seconds")(
          "preprocessTimeLimit", po::value<int>()->default_value(INT_MAX),
          "the time limit for preprocessing in seconds")(
          "logFile,l", po::value<std::string>()->default_value(""),
          "issue log file name");

  clock_t start_time = clock();

  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  po::notify(vm);

  std::string log_file = vm["logFile"].as<std::string>();
  Logger logger = Logger(log_file);
  std::string base_folder = vm["inputFolder"].as<std::string>();
  boost::filesystem::path p(vm["inputFile"].as<std::string>());
  std::string output_file = vm["output"].as<std::string>();
  int output_screen = vm["outputScreen"].as<int>();
  bool evaluation_mode = vm["evaluationMode"].as<bool>();
  int max_simulation_time = vm["simulationTime"].as<int>();
  int plan_time_limit = vm["planTimeLimit"].as<int>();
  int preprocess_time_limit = vm["preprocessTimeLimit"].as<int>();

  boost::filesystem::path dir = p.parent_path();
  base_folder = dir.string();
  if (base_folder.size() > 0 && base_folder.back() != '/') {
    base_folder += "/";
  }

  std::string input_json_file = vm["inputFile"].as<std::string>();
  json data;
  std::ifstream f(input_json_file);
  try {
    data = json::parse(f);
  } catch (json::parse_error error) {
    std::cerr << "Failed to load " << input_json_file << std::endl;
    std::cerr << "Message: " << error.what() << std::endl;
    exit(1);
  }

  std::string map_file = read_param_json<std::string>(data, "mapFile");
  auto map_path = boost::filesystem::path(map_file);
  auto config_dir = p.remove_filename();
  auto x = config_dir / map_path;

  
  //std::cout << map_file << std::endl;
  int team_size = read_param_json<int>(data, "teamSize");
  int num_task_real = read_param_json<int>(data, "numTasksReveal", 1);
  std::string agent_file =
      base_folder + read_param_json<std::string>(data, "agentFile");
  std::string task_file =
      base_folder + read_param_json<std::string>(data, "taskFile");

  std::vector<int> agents = read_int_vec(agent_file, team_size);

  // Load tasks
  std::ifstream task_file_stream(task_file.c_str());
  task_generator::TaskGenerator task_generator(task_file_stream);
  task_file_stream.close();

  // Build and Assemble the system
  SharedEnvironment state(team_size);
  Grid grid(x.string());
  ActionModelWithRotate model(grid);
  PerfectSimulator simulator(model, grid);
  task_assigner::TaskAssigner task_assigner;
  planner::MAPFPlanner planner(grid);
  planner::MAPFPlannerWrapper wrapped_planner(&planner, &logger);
  execution_policy::MAPFExecutionPolicy execution_policy(&wrapped_planner);
  base_system::BaseSystem<task_generator::TaskGenerator,
                          task_assigner::TaskAssigner,
                          execution_policy::MAPFExecutionPolicy,
                          planner::MAPFPlannerWrapper, PerfectSimulator>
      system(&task_generator, &task_assigner, &execution_policy,
             &wrapped_planner, &simulator, &logger);

  //signal(SIGINT, sigint_handler);

  system.simulate(&state, max_simulation_time);

  // collect metrics from each of the components of the competition system.
  base_system::metrics_t system_metrics = system.get_metrics();
  planner::planner_metrics_t planner_metrics = wrapped_planner.get_metrics();
  task_generator::task_generator_metrics_t task_generator_metrics =
      task_generator.get_metrics();

  //base_system::save_results(output_file, output_screen);
  save_results(output_file, system_metrics, planner_metrics, task_generator_metrics);
}
