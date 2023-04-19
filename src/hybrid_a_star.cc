#include <cmath>

#include "hybrid_a_star.h"

HybridAStar::HybridAStar(const WarmStartConfig& warm_start_config,
                         const VehicleParam& vehicle_param)
    : warm_start_config_(warm_start_config), vehicle_param_(vehicle_param) {
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, warm_start_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(warm_start_config_);
  next_node_num_ = warm_start_config_.next_node_num;
  max_steer_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;
  step_size_ = warm_start_config_.step_size;
  xy_grid_resolution_ = warm_start_config_.xy_grid_resolution;
  delta_t_ = warm_start_config_.delta_t;
  traj_forward_penalty_ = warm_start_config_.traj_forward_penalty;
  traj_back_penalty_ = warm_start_config_.traj_back_penalty;
  traj_gear_switch_penalty_ = warm_start_config_.traj_gear_switch_penalty;
  traj_steer_penalty_ = warm_start_config_.traj_steer_penalty;
  traj_steer_change_penalty_ = warm_start_config_.traj_steer_change_penalty;
  max_kappa_ =
      std::tan(vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio) /
      vehicle_param_.wheel_base;
  min_radius_ = 1.0 / max_kappa_;
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  // std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
  //     std::make_shared<ReedSheppPath>();
  // if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
  //                                         reeds_shepp_to_check)) {
  //   std::cout << "ShortestRSP failed";
  //   return false;
  // }

  // if (!RSPCheck(reeds_shepp_to_check)) {
  //   return false;
  // }

  // std::cout << "Reach the end configuration with Reed Sharp";
  // // load the whole RSP as nodes and add to the close set
  // final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);

  std::vector<Pos3d> curve_path = math::GetTrajFromCurvePathsConnect(
      current_node->GetPose(), end_node_->GetPose(), min_radius_,
      xy_grid_resolution_);
  if (!IsPathVaild(curve_path)) {
    return false;
  }

  final_node_ = GenerateFinalNode(curve_path, current_node);
  return true;
}

std::shared_ptr<Node3d> HybridAStar::GenerateFinalNode(
    const std::vector<Pos3d>& curve_path,
    std::shared_ptr<Node3d> current_node) {
  std::vector<double> traversed_x;
  std::vector<double> traversed_y;
  std::vector<double> traversed_phi;
  for (const auto& pose : curve_path) {
    traversed_x.push_back(pose.x);
    traversed_y.push_back(pose.y);
    traversed_phi.push_back(pose.phi);
  }

  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(
      new Node3d(traversed_x, traversed_y, traversed_phi, XYbounds_,
                 warm_start_config_.grid_a_star_xy_resolution));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

bool HybridAStar::IsPathVaild(const std::vector<Pos3d>& curve_path) {
  if (curve_path.empty()) {
    return false;
  }

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  for (const auto& pose : curve_path) {
    if (pose.x > XYbounds_[1] || pose.x < XYbounds_[0] ||
        pose.y > XYbounds_[3] || pose.y < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box =
        Node3d::GetBoundingBox(vehicle_param_, pose.x, pose.y, pose.phi);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        if (math::HasOverlap(bounding_box, linesegment)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, warm_start_config_.grid_a_star_xy_resolution));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        if (math::HasOverlap(bounding_box, linesegment)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, warm_start_config_.grid_a_star_xy_resolution));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 warm_start_config_.grid_a_star_xy_resolution));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      std::cout << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      std::cout << "states sizes are not equal";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  if (!GetTemporalProfile(result)) {
    std::cout << "GetSpeedProfile from Hybrid Astar path fails";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    std::cout << "state sizes not equal, "
              << "result->x.size(): " << result->x.size() << "result->y.size()"
              << result->y.size() << "result->phi.size()" << result->phi.size()
              << "result->v.size()" << result->v.size();
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    std::cout << "control sizes not equal or not right";
    std::cout << " acceleration size: " << result->a.size();
    std::cout << " steer size: " << result->steer.size();
    std::cout << " x size: " << result->x.size();
    return false;
  }
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    std::cout
        << "result size check when generating speed and acceleration fail";
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    std::cout
        << "states sizes are not equal when do trajectory partitioning of "
           "Hybrid A Star result";
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector{x[1] - x[0], y[1] - y[0]};
  double tracking_angle = math::GetVecAngle(init_tracking_vector);
  bool current_gear =
      std::abs(math::NormalizeAngle(tracking_angle - heading_angle)) < (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector{x[i + 1] - x[i], y[i + 1] - y[i]};
    tracking_angle = math::GetVecAngle(tracking_vector);
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    if (!GenerateSpeedAcceleration(&result)) {
      std::cout << "GenerateSpeedAcceleration fail";
      return false;
    }
  }

  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    std::cout << "TrajectoryPartition fail";
    return false;
  }
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  *result = stitched_result;
  return true;
}

bool HybridAStar::Plan(
    const Pos3d& start_pose, const Pos3d& end_pose,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;
  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      Vec2d start = obstacle_vertices[i], end = obstacle_vertices[i + 1];
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double length = std::hypot(dx, dy);
      Vec2d unit_direction =
          (length <= kMathEpsilon ? Vec2d{0, 0}
                                  : Vec2d{dx / length, dy / length});
      const double heading = math::GetVecAngle(unit_direction);
      LineSegment2d line_segment =
          LineSegment2d{start, end, unit_direction, heading, length};
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  // load XYbounds
  XYbounds_ = XYbounds;
  // load nodes and obstacles
  start_node_.reset(new Node3d({start_pose.x}, {start_pose.y}, {start_pose.phi},
                               XYbounds_,
                               warm_start_config_.grid_a_star_xy_resolution));
  end_node_.reset(new Node3d({end_pose.x}, {end_pose.y}, {end_pose.phi},
                             XYbounds_,
                             warm_start_config_.grid_a_star_xy_resolution));
  if (!ValidityCheck(start_node_)) {
    std::cout << "start_node in collision with obstacles";
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    std::cout << "end_node in collision with obstacles";
    return false;
  }
  // double map_time = Clock::NowInSeconds();
  grid_a_star_heuristic_generator_->GenerateDpMap(
      end_pose.x, end_pose.y, XYbounds_, obstacles_linesegments_vec_);
  // std::cout << "map time " << Clock::NowInSeconds() - map_time;
  //  load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
  // Hybrid A* begins
  size_t explored_node_num = 0;
  // double astar_start_time = Clock::NowInSeconds();
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    // const double rs_start_time = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node)) {
      break;
    }
    // const double rs_end_time = Clock::NowInSeconds();
    // rs_time += rs_end_time - rs_start_time;
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        // const double start_time = Clock::NowInSeconds();
        CalculateNodeCost(current_node, next_node);
        // const double end_time = Clock::NowInSeconds();
        // heuristic_time += end_time - start_time;
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  if (final_node_ == nullptr) {
    std::cout << "Hybrid A searching return null ptr(open_set ran out)";
    return false;
  }
  if (!GetResult(result)) {
    std::cout << "GetResult failed";
    return false;
  }
  std::cout << "explored node num is " << explored_node_num;
  return true;
}
