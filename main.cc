#include <iostream>
#include <fstream>
#include <memory>

//#include "src/reeds_shepp_path.h"
#include "read_conf.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"
#include "result_plot/cpp_plot.h"

namespace plt = matplotlibcpp;

int main()
{
  // test reedshepp
  //  std::vector<double> XYbounds_;
  //  XYbounds_.push_back(-100.0);
  //  XYbounds_.push_back(100.0);
  //  XYbounds_.push_back(-100.0);
  //  XYbounds_.push_back(100.0);

  // VehicleParam vehicle_param_;
  // WarmStartConfig warm_start_config_;

  // std::unique_ptr<ReedShepp> reedshepp_test = std::unique_ptr<ReedShepp>(
  //              new ReedShepp(vehicle_param_, warm_start_config_));
  // std::shared_ptr<Node3d> start_node = std::shared_ptr<Node3d>(new Node3d(
  //   0.0, 0.0, 10.0 * M_PI / 180.0, XYbounds_, 0.1));
  // std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
  //   7.0, -8.0, 50.0 * M_PI / 180.0, XYbounds_, 0.1));
  // std::shared_ptr<ReedSheppPath> optimal_path =
  //   std::shared_ptr<ReedSheppPath>(new ReedSheppPath());

  // if (!reedshepp_test->ShortestRSP(start_node, end_node, optimal_path)) {
  //  std::cout << "generating short RSP not successful";
  // }else{
  //   std::cout << "size:" <<optimal_path->x.size() << std::endl;
  //   std::vector<double> x, y, z;
  //   for(int i = 0 ;i<optimal_path->x.size();++i){
  //   std::cout << optimal_path->x[i] <<","
  //             << optimal_path->y[i] <<","
  //             << optimal_path->phi[i] <<std::endl;

  //   x.push_back(optimal_path->x[i]);
  //   y.push_back(optimal_path->y[i]);
  //   z.push_back(optimal_path->phi[i]);
  //   Show_Result(x, y, z);

  //   }
  // }

  // test2 hybrid_a_star
  // set scence;
  std::vector<Vec2d> obs1 = {Vec2d{13, 8},
                             Vec2d{1.5, 8},
                             Vec2d{1.5, 0},
                             Vec2d{13, 0}};

  std::vector<Vec2d> obs2 = {Vec2d{-1.5, 8},
                             Vec2d{-13, 8},
                             Vec2d{-13, 0},
                             Vec2d{-1.5, 0}};

  std::vector<Vec2d> obs3 = {Vec2d{1.5, 2},
                             Vec2d{-1.5, 2},
                             Vec2d{-1.5, 0},
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
  if(!ReadHybridAStarParam(warm_start_config)){
    std::cout << "read failed." <<std::endl;
    return 0;
  }

  VehicleParam vehicle_param;
  if(!ReadVehicleParam(vehicle_param)){
    std::cout << "read failed." <<std::endl;
    return 0;
  }


  std::unique_ptr<HybridAStar> hybrid_test = std::unique_ptr<HybridAStar>(
      new HybridAStar(warm_start_config, vehicle_param));
  hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds,
                    obstacle, &result);

  PlotObstacles(obstacle);
  PlotVehicleTraj(result.x, result.y, result.phi);
  plt::show();

  std::cout << "test" << std::endl;
  return 0;
}