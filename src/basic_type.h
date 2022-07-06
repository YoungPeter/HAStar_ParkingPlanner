#ifndef BASIC_TYPE
#define BASIC_TYPE

struct VehicleParam {
  double front_edge_to_center;
  double back_edge_to_center;
  double left_edge_to_center;
  double right_edge_to_center;
  double length;
  double width;
  double height;
  double min_turn_radius;
  double max_acceleration;
  double max_deceleration;
  double max_steer_angle;
  double max_steer_angle_rate;
  double steer_ratio;
  double wheel_base;
  double wheel_rolling_radius;
  double max_abs_speed_when_stopped;
  double brake_deadzone;
  double throttle_deadzone;
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

#endif