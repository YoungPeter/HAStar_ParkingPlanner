#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "read_conf.h"

namespace {
std::vector<std::string> Stringsplit(const std::string& str,
                                     const std::string& splits) {
  std::vector<std::string> res;
  if (str == "") {
    return res;
  }
  std::string strs = str + splits;
  size_t pos = strs.find(splits);
  int step = splits.size();

  while (pos != strs.npos) {
    std::string temp = strs.substr(0, pos);
    res.push_back(temp);
    strs = strs.substr(pos + step, strs.size());
    pos = strs.find(splits);
  }
  return res;
}

Pos3d GetParkingScenarioPose(const std::string& str) {
  std::vector<std::string> s = Stringsplit(str, /*spilt=*/",");
  Pos3d pose;
  if (s.size() != 3) {
    std::cout << "read pose failed" << std::endl;
    return pose;
  }
  pose.x = std::stod(s[0]);
  pose.y = std::stod(s[1]);
  pose.phi = std::stod(s[2]);
  return pose;
}

std::vector<double> GetParkingScenarioBoundary(const std::string& str) {
  std::vector<std::string> s = Stringsplit(str, /*spilt=*/",");
  std::vector<double> boundary;
  if (s.size() != 4) {
    std::cout << "read boundary failed" << std::endl;
    return boundary;
  }

  for (const auto ss : s) {
    boundary.emplace_back(std::stod(ss));
  }
  return boundary;
}

std::vector<std::vector<Vec2d>> GetParkingScenarioObstacle(
    const std::string& str) {
  std::vector<std::vector<Vec2d>> obstacles;
  const std::regex reg("-[0-9]+(.[0-9]+)?|[0-9]+(.[0-9]+)?");
  std::vector<std::string> s = Stringsplit(str, /*spilt=*/";");
  for (auto ss : s) {
    std::vector<Vec2d> obstacle;
    std::vector<std::string> obstacle_str;
    const std::sregex_iterator end;
    for (std::sregex_iterator iter(ss.cbegin(), ss.cend(), reg); iter != end;
         ++iter) {
      obstacle_str.push_back(iter->str());
    }
    for (int i = 0; i < obstacle_str.size(); i = i + 2) {
      Vec2d point{std::stod(obstacle_str[i]), std::stod(obstacle_str[i + 1])};
      obstacle.emplace_back(point);
    }
    obstacles.emplace_back(obstacle);
  }
  return obstacles;
}
}  // namespace

bool ReadParkingScenario(ParkingScenario& parking_scenario) {
  std::ifstream file;
  file.open("config\\parking_scenario.txt", std::ios_base::in);
  if (!file.is_open()) {
    return false;
  }

  std::string s;
  int n_row = 0;
  while (getline(file, s)) {
    size_t pos = s.find(':');
    std::string value_str = s.substr(pos + 2);
    switch (n_row++) {
      case 0:
        parking_scenario.start_pos = GetParkingScenarioPose(value_str);
        break;
      case 1:
        parking_scenario.end_pos = GetParkingScenarioPose(value_str);
        break;
      case 2:
        parking_scenario.boundary = GetParkingScenarioBoundary(value_str);
        break;
      case 3:
        parking_scenario.obstacles = GetParkingScenarioObstacle(value_str);
        break;
      default:
        std::cout << "The number of rows is out of range." << std::endl;
        break;
    }
  }
  return true;
}

bool ReadHybridAStarParam(WarmStartConfig& warm_start_config) {
  std::ifstream file;
  file.open(
      "config\\hya_param."
      "conf",
      std::ios_base::in);
  if (!file.is_open()) {
    return false;
  }

  std::string s;
  int n_row = 0;
  while (getline(file, s)) {
    size_t pos = s.find(':');
    std::string value_str = s.substr(pos + 2);
    double value = std::stod(value_str);
    switch (n_row) {
      case 0:
        warm_start_config.xy_grid_resolution = value;
        break;
      case 1:
        warm_start_config.phi_grid_resolution = value;
        break;
      case 2:
        warm_start_config.next_node_num = value;
        break;
      case 3:
        warm_start_config.step_size = value;
        break;
      case 4:
        warm_start_config.traj_forward_penalty = value;
        break;
      case 5:
        warm_start_config.traj_back_penalty = value;
        break;
      case 6:
        warm_start_config.traj_gear_switch_penalty = value;
        break;
      case 7:
        warm_start_config.traj_steer_penalty = value;
        break;
      case 8:
        warm_start_config.traj_steer_change_penalty = value;
        break;
      case 9:
        warm_start_config.grid_a_star_xy_resolution = value;
        break;
      case 10:
        warm_start_config.node_radius = value;
        break;
      case 11:
        warm_start_config.delta_t = value;
        break;
      default:
        std::cout << "The number of rows is out of range." << std::endl;
        break;
    }
    n_row++;
  }
  file.close();
  return true;
}

bool ReadVehicleParam(VehicleParam& vehicle_param) {
  std::ifstream file;
  file.open(
      "config\\vehicle_param."
      "conf",
      std::ios_base::in);
  if (!file.is_open()) {
    return false;
  }

  std::string s;
  int n_row = 0;
  while (getline(file, s)) {
    size_t pos = s.find(':');
    std::string value_str = s.substr(pos + 2);
    double value = std::stod(value_str);
    switch (n_row) {
      case 0:
        vehicle_param.front_edge_to_center = value;
        break;
      case 1:
        vehicle_param.back_edge_to_center = value;
        break;
      case 2:
        vehicle_param.length = value;
        break;
      case 3:
        vehicle_param.width = value;
        break;
      case 4:
        vehicle_param.max_steer_angle = value;
        break;
      case 5:
        vehicle_param.steer_ratio = value;
        break;
      case 6:
        vehicle_param.wheel_base = value;
        break;
      default:
        std::cout << "The number of rows is out of range." << std::endl;
        break;
    }
    n_row++;
  }
  file.close();
  return true;
}