#ifndef BASIC_TYPE
#define BASIC_TYPE

#include <vector>

struct VehicleParam {
  double front_edge_to_center;
  double back_edge_to_center;
  double length;
  double width;
  double max_steer_angle;
  double steer_ratio;
  double wheel_base;
};

struct WarmStartConfig {
  // Hybrid a star for warm start
  double xy_grid_resolution;
  double phi_grid_resolution;
  int next_node_num;
  double step_size;
  double traj_forward_penalty;
  double traj_back_penalty;
  double traj_gear_switch_penalty;
  double traj_steer_penalty;
  double traj_steer_change_penalty;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution;
  double node_radius;
  double delta_t;
  // PiecewiseJerkSpeedOptimizerConfig s_curve_config = 17;
};

struct Vec2d {
  double x_ = 0.0;
  double y_ = 0.0;
};

struct Pos3d {
  double x;
  double y;
  double phi;
};

struct LineSegment2d {
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};

struct CurvePath {
  double x;
  double y;
  double phi;
  double dist;
  double radius;
};

struct Box2d {
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
};

struct ParkingScenario {
  Pos3d start_pos;
  Pos3d end_pos;
  std::vector<double> boundary;
  std::vector<std::vector<Vec2d>> obstacles;
};
#endif