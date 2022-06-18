#ifndef CPP_PLOT
#define CPP_PLOT

#include <cmath>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void PlotVehicle(double x, double y, double phi);

void PlotObstacles(std::vector<std::vector<Vec2d>> obstacles_vertices_vec)
{
  if (obstacles_vertices_vec.empty())
  {
    return;
  }

  for (const auto &obstacles_vertices : obstacles_vertices_vec)
  {
    size_t vertices_num = obstacles_vertices.size();
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    for (size_t i = 0; i < vertices_num; ++i)
    {
      obstacles_x.emplace_back(obstacles_vertices[i].x_);
      obstacles_y.emplace_back(obstacles_vertices[i].y_);
      obstacles_x.emplace_back(obstacles_vertices[i + 1].x_);
      obstacles_y.emplace_back(obstacles_vertices[i + 1].y_);
      plt::plot(obstacles_x, obstacles_y, "blue");
    }

    obstacles_x.emplace_back(obstacles_vertices.back().x_);
    obstacles_y.emplace_back(obstacles_vertices.back().y_);
    obstacles_x.emplace_back(obstacles_vertices[0].x_);
    obstacles_y.emplace_back(obstacles_vertices[0].y_);
    plt::plot(obstacles_x, obstacles_y, "blue");
  }
}

void PlotTrajectory(const std::vector<double> &x, const std::vector<double> &y)
{
  plt::plot(x, y, "red");
  plt::axis("equal");
}

void PlotVehicleTraj(const std::vector<double> &traj_x, const std::vector<double> &traj_y, const std::vector<double> &traj_phi)
{
  for (size_t i = 0; i < traj_x.size(); ++i)
  {
    PlotVehicle(traj_x[i], traj_y[i], traj_phi[i]);
  }

  PlotTrajectory(traj_x, traj_y);
}

void PlotVehicle(double x, double y, double phi){
  const double cos_heading = std::cos(phi);
  const double sin_heading = std::sin(phi);
  VehicleParam veh_para;

  //front dx dy
  const double front_dx = cos_heading * veh_para.front_edge_to_center;
  const double front_dy = sin_heading * veh_para.front_edge_to_center;
  
  //back dx dy
  const double back_dx = cos_heading * veh_para.back_edge_to_center;
  const double back_dy = sin_heading * veh_para.back_edge_to_center;

  //width dx dy
  const double dx2 = sin_heading * veh_para.width/2.0;
  const double dy2 = -cos_heading * veh_para.width/2.0;

  std::vector<Vec2d> corners;  
  corners.emplace_back(Vec2d{x + front_dx + dx2, y + front_dy + dy2});
  corners.emplace_back(Vec2d{x + front_dx - dx2, y + front_dy - dy2});
  corners.emplace_back(Vec2d{x - back_dx - dx2, y - back_dy - dy2});
  corners.emplace_back(Vec2d{x - back_dx + dx2, y - back_dy + dy2});

  std::vector<double> vehicle_corner_x, vehicle_corner_y;
  for(const auto& coner:corners){
    vehicle_corner_x.emplace_back(coner.x_);
    vehicle_corner_y.emplace_back(coner.y_);
  }
  vehicle_corner_x.emplace_back(corners.front().x_);
  vehicle_corner_y.emplace_back(corners.front().y_);
  
  plt::plot(vehicle_corner_x, vehicle_corner_y, "green");
  plt::axis("equal");
}

#endif