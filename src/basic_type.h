#ifndef BASIC_TYPE
#define BASIC_TYPE

struct VehicleParam
{
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;
  double length = 4.933;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 8.55211;
  double steer_ratio = 16;
  double wheel_base = 2.8448;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;
  double brake_deadzone = 14.5;
  double throttle_deadzone = 15.4;
};

struct WarmStartConfig
{
  // Hybrid a star for warm start
  double xy_grid_resolution =  0.2;
  double phi_grid_resolution = 0.0;
  int next_node_num = 10;
  double step_size = 0.5;
  double traj_forward_penalty = 0.0;
  double traj_back_penalty = 0.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty =100.0;
  double traj_steer_change_penalty =  10.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution =  0.1;
  double node_radius =  0.5;
  double delta_t = 1.0;
  //PiecewiseJerkSpeedOptimizerConfig s_curve_config = 17;

};

struct Vec2d{
  double x_ = 0.0;
  double y_ = 0.0;
};

struct LineSegment2d{
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};


struct Box2d{
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