#include <cmath>
#include <iostream>

#include "grid_search.h"
#include "math_utils.h"

GridSearch::GridSearch(const WarmStartConfig& warm_start_config) {
  xy_grid_resolution_ = warm_start_config.grid_a_star_xy_resolution;
  node_radius_ = warm_start_config.node_radius;
}

double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }
  for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
    for (const LineSegment2d& linesegment : obstacle_linesegments) {
      if (math::DistanceTo(linesegment, {node->GetGridX(), node->GetGridY()}) <
          node_radius_) {
        return false;
      }
    }
  }
  return true;
}

std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = 1.41421356;
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::vector<std::vector<double>> directions = {
      {0.0, 1.0, 1.0},  {1.0, 1.0, diagonal_distance},
      {1.0, 0.0, 1.0},  {1.0, -1.0, diagonal_distance},
      {0.0, -1.0, 1.0}, {-1.0, -1.0, diagonal_distance},
      {-1.0, 0.0, 1.0}, {-1.0, 1.0, diagonal_distance}};
  for (const auto dir : directions) {
    std::shared_ptr<Node2d> temp_node = std::make_shared<Node2d>(
        current_node_x + dir[0], current_node_y + dir[1], XYbounds_);
    temp_node->SetPathCost(current_node_path_cost + dir[2]);
    next_nodes.emplace_back(temp_node);
  }
  return next_nodes;
}

bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::vector<double>& XYbounds,
    const std::vector<std::vector<LineSegment2d>>& obstacles_linesegments_vec) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;
  // XYbounds with xmin, xmax, ymin, ymax
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    dp_map_.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  std::cout << "explored node num is " << explored_node_num;
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
  std::string index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}
