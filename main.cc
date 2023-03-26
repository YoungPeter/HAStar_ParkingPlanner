#include <fstream>
#include <iostream>
#include <memory>

#include "read_conf.h"
#include "result_plot/cpp_plot.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"

namespace plt = matplotlibcpp;

int main() {
  std::vector<Vec2d> obs1 = {Vec2d{13, 8}, Vec2d{1.5, 8}, Vec2d{1.5, 0},
                             Vec2d{13, 0}};

  std::vector<Vec2d> obs2 = {Vec2d{-1.5, 8}, Vec2d{-13, 8}, Vec2d{-13, 0},
                             Vec2d{-1.5, 0}};

  std::vector<Vec2d> obs3 = {Vec2d{1.5, 2}, Vec2d{-1.5, 2}, Vec2d{-1.5, 0},
                             Vec2d{1.5, 0}};
  std::vector<std::vector<Vec2d>> obstacle = {obs1, obs2, obs3};
  std::vector<double> XYbounds = {-15, 15, 0, 40};
  double sx = -6.0;
  double sy = 10.5;
  double sphi = M_PI;
  double ex = 0;
  double ey = 3.5;
  double ephi = M_PI_2;
  HybridAStartResult result;

  WarmStartConfig warm_start_config;
  if (!ReadHybridAStarParam(warm_start_config)) {
    std::cout << "read failed." << std::endl;
    return 0;
  }

  VehicleParam vehicle_param;
  if (!ReadVehicleParam(vehicle_param)) {
    std::cout << "read failed." << std::endl;
    return 0;
  }

  std::unique_ptr<HybridAStar> hybrid_test = std::unique_ptr<HybridAStar>(
      new HybridAStar(warm_start_config, vehicle_param));
  hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds, obstacle, &result);

  PlotObstacles(obstacle);
  PlotVehicleTraj(result.x, result.y, result.phi);
  plt::show();

  std::cout << "test" << std::endl;
  return 0;
}