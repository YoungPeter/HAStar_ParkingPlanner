#include <iostream>
#include <memory>

//#include "src/reeds_shepp_path.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"


int main(){
    
    //test node3d
    //std::unique_ptr<Node3d> node = std::unique_ptr<Node3d>(new Node3d(1.0, 0.0, 0.0));
    //std::cout<<"x: "<< node->GetX() << std::endl;

    //test reedshepp
    // std::vector<double> XYbounds_;
    // XYbounds_.push_back(-100.0);
    // XYbounds_.push_back(100.0);
    // XYbounds_.push_back(-100.0);
    // XYbounds_.push_back(100.0);
    
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
    //   for(int i = 0 ;i<optimal_path->x.size();++i){
    //   std::cout << optimal_path->x[i] <<"," 
    //             << optimal_path->y[i] <<"," 
    //             << optimal_path->phi[i] <<std::endl;
    //   }
    // }

  double sx = -15.0;
  double sy = 0.0;
  double sphi = 0.0;
  double ex = 15.0;
  double ey = 0.0;
  double ephi = 0.0;
  std::vector<std::vector<Vec2d>> obstacles_list;
  HybridAStartResult result;
  Vec2d obstacle_vertice_a{1.0, 0.0};
  Vec2d obstacle_vertice_b{-1.0, 0.0};
  std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
  // load xy boundary into the Plan() from configuration(Independent from frame)
  std::vector<double> XYbounds_;
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);

  obstacles_list.emplace_back(obstacle);
  const WarmStartConfig warm_start_config;

  HybridAStar hybrid_test(warm_start_config);

  // std::unique_ptr<HybridAStar> hybrid_test = std::unique_ptr<HybridAStar>(
  //       new HybridAStar(warm_start_config));
  // hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
  //                                obstacles_list, &result);
  std::cout<<"test"<<std::endl;
  return 0;
}