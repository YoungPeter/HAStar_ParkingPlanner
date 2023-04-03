#include <fstream>
#include <iostream>
#include <memory>

// #include "src/reeds_shepp_path.h"
#include "read_conf.h"
#include "result_plot/cpp_plot.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"

namespace plt = matplotlibcpp;

int main() {
  ParkingScenario parking_scenario;
  if (!ReadParkingScenario(parking_scenario)) {
    std::cout << "read parking scenario failed." << std::endl;
    return 0;
  }

  WarmStartConfig warm_start_config;
  if (!ReadHybridAStarParam(warm_start_config)) {
    std::cout << "read hybrid a star param failed." << std::endl;
    return 0;
  }

  VehicleParam vehicle_param;
  if (!ReadVehicleParam(vehicle_param)) {
    std::cout << "read vehicle param failed." << std::endl;
    return 0;
  }

  HybridAStartResult result;
  std::unique_ptr<HybridAStar> hybrid_test = std::unique_ptr<HybridAStar>(
      new HybridAStar(warm_start_config, vehicle_param));
  hybrid_test->Plan(parking_scenario.start_pos, parking_scenario.end_pos,
                    parking_scenario.boundary, parking_scenario.obstacles,
                    &result);

  PlotObstacles(parking_scenario.obstacles);
  PlotVehicleTraj(vehicle_param, result.x, result.y, result.phi);
  plt::show();
  return 0;
}