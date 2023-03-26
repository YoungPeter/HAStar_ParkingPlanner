#include <cmath>
#include <limits>

#include "math_utils.h"

namespace math {

double sign(double x) { return x >= 0 ? 1 : -1; }

double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length) {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double proj = x0 * dx + y0 * dy;
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

double CalProjectInX(const Pos3d &pos, const Vec2d &point) {
  return (point.x_ - pos.x) * std::cos(pos.phi) +
         (point.y_ - pos.y) * std::sin(pos.phi);
}

double CalProjectInX(const Pos3d &pos1, const Pos3d &pos2) {
  return (pos2.x - pos1.x) * std::cos(pos1.phi) +
         (pos2.y - pos1.y) * std::sin(pos1.phi);
}

double CalProjectInY(const Pos3d &pos, const Vec2d &point) {
  return (point.y_ - pos.y) * std::cos(pos.phi) -
         (point.x_ - pos.x) * std::sin(pos.phi);
}

double CalProjectInY(const Pos3d &pos1, const Pos3d &pos2) {
  return (pos2.y - pos1.y) * std::cos(pos1.phi) -
         (pos2.x - pos1.x) * std::sin(pos1.phi);
}

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  Vec2d v1{end_point_1.x_ - start_point.x_, end_point_1.y_ - start_point.y_};
  Vec2d v2{end_point_2.x_ - start_point.x_, end_point_2.y_ - start_point.y_};

  return v1.x_ * v2.y_ - v1.y_ * v2.x_;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point) {
  if (line_segment2d.length_ <= 1e-10) {
    return std::hypot(
        point.x_ - line_segment2d.start_.x_,
        point.y_ - line_segment2d.start_.y_);  // point.DistanceTo(start_);
  }
  const double x0 = point.x_ - line_segment2d.start_.x_;
  const double y0 = point.y_ - line_segment2d.start_.y_;
  const double proj = x0 * line_segment2d.unit_direction_.x_ +
                      y0 * line_segment2d.unit_direction_.y_;
  if (proj <= 0.0) {
    return std::hypot(x0, y0);
  }
  if (proj >= line_segment2d.length_) {
    return std::hypot(point.x_ - line_segment2d.end_.x_,
                      point.y_ - line_segment2d.end_.y_);
  }
  return std::abs(x0 * line_segment2d.unit_direction_.y_ -
                  y0 * line_segment2d.unit_direction_.x_);
}

double GetVecAngle(Vec2d v) { return std::atan2(v.y_, v.x_); }

double DistanceTo(const Box2d &bounding_box, const Vec2d &point) {
  const double x0 = point.x_ - bounding_box.center_.x_;
  const double y0 = point.y_ - bounding_box.center_.y_;
  const double dx = std::abs(x0 * bounding_box.cos_heading_ +
                             y0 * bounding_box.sin_heading_) -
                    bounding_box.half_length_;
  const double dy = std::abs(x0 * bounding_box.sin_heading_ -
                             y0 * bounding_box.cos_heading_) -
                    bounding_box.half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

double DistanceTo(const Box2d &bounding_box,
                  const LineSegment2d &line_segment) {
  if (line_segment.length_ <= kMathEpsilon) {
    return DistanceTo(bounding_box, line_segment.start_);
  }
  const double ref_x1 = line_segment.start_.x_ - bounding_box.center_.x_;
  const double ref_y1 = line_segment.start_.y_ - bounding_box.center_.y_;
  double x1 =
      ref_x1 * bounding_box.cos_heading_ + ref_y1 * bounding_box.sin_heading_;
  double y1 =
      ref_x1 * bounding_box.sin_heading_ - ref_y1 * bounding_box.cos_heading_;
  double box_x = bounding_box.half_length_;
  double box_y = bounding_box.half_width_;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const double ref_x2 = line_segment.end_.x_ - bounding_box.center_.x_;
  const double ref_y2 = line_segment.end_.y_ - bounding_box.center_.y_;
  double x2 =
      ref_x2 * bounding_box.cos_heading_ + ref_y2 * bounding_box.sin_heading_;
  double y2 =
      ref_x2 * bounding_box.sin_heading_ - ref_y2 * bounding_box.cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                             line_segment.length_);
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length_);
      case 2:
        return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                         line_segment.length_)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length_);
      case -1:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length_);
      case -4:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length_)
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                          line_segment.length_));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length_);
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                   line_segment.length_);
      case -3:
        return 0.0;
    }
  }
  return 0.0;
}

bool IsPointIn(const Box2d &bounding_box, const Vec2d &point) {
  const double x0 = point.x_ - bounding_box.center_.x_;
  const double y0 = point.y_ - bounding_box.center_.y_;
  const double dx =
      std::abs(x0 * bounding_box.cos_heading_ + y0 * bounding_box.sin_heading_);
  const double dy = std::abs(-x0 * bounding_box.sin_heading_ +
                             y0 * bounding_box.cos_heading_);
  return dx <= bounding_box.half_length_ + 1e-10 &&
         dy <= bounding_box.half_width_ + 1e-10;
}

bool HasOverlap(const Box2d &bounding_box, const LineSegment2d &line_segment) {
  if (line_segment.length_ <= 1e-10) {
    return IsPointIn(bounding_box, line_segment.start_);
  }

  std::vector<Vec2d> corners;
  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();

  const double dx1 = bounding_box.cos_heading_ * bounding_box.half_length_;
  const double dy1 = bounding_box.sin_heading_ * bounding_box.half_length_;
  const double dx2 = bounding_box.sin_heading_ * bounding_box.half_width_;
  const double dy2 = -bounding_box.cos_heading_ * bounding_box.half_width_;

  corners.emplace_back(Vec2d{bounding_box.center_.x_ + dx1 + dx2,
                             bounding_box.center_.y_ + dy1 + dy2});
  corners.emplace_back(Vec2d{bounding_box.center_.x_ + dx1 - dx2,
                             bounding_box.center_.y_ + dy1 - dy2});
  corners.emplace_back(Vec2d{bounding_box.center_.x_ - dx1 - dx2,
                             bounding_box.center_.y_ - dy1 - dy2});
  corners.emplace_back(Vec2d{bounding_box.center_.x_ - dx1 + dx2,
                             bounding_box.center_.y_ - dy1 + dy2});
  for (auto &corner : corners) {
    max_x_ = std::fmax(corner.x_, max_x_);
    min_x_ = std::fmin(corner.x_, min_x_);
    max_y_ = std::fmax(corner.y_, max_y_);
    min_y_ = std::fmin(corner.y_, min_y_);
  }

  if (std::fmax(line_segment.start_.x_, line_segment.end_.x_) < min_x_ ||
      std::fmin(line_segment.start_.x_, line_segment.end_.x_) > max_x_ ||
      std::fmax(line_segment.start_.y_, line_segment.end_.y_) < min_y_ ||
      std::fmin(line_segment.start_.y_, line_segment.end_.y_) > max_y_) {
    return false;
  }
  return DistanceTo(bounding_box, line_segment) <= 1e-10;
}

Pos3d CalEndPosWithACurvePath(const Pos3d &start_pos, double dist,
                              double radius) {
  Pos3d end_pos;

  if (std::abs(radius) < kMathEpsilon) {
    end_pos.x = start_pos.x + std::cos(start_pos.phi) * dist;
    end_pos.y = start_pos.y + std::sin(start_pos.phi) * dist;
    end_pos.phi = start_pos.phi;
  } else {
    double theta = start_pos.phi - sign(radius) * M_PI / 2.0;
    end_pos.x = start_pos.x +
                std::cos(dist / radius + theta) * std::abs(radius) -
                std::cos(theta) * std::abs(radius);
    end_pos.y = start_pos.y +
                std::sin(dist / radius + theta) * std::abs(radius) -
                std::sin(theta) * std::abs(radius);
    end_pos.phi = start_pos.phi + dist / radius;
  }

  return end_pos;
}

Pos3d CalEndPosWithACurvePath(CurvePath curve_path) {
  return CalEndPosWithACurvePath(
      Pos3d{curve_path.x, curve_path.y, curve_path.phi}, curve_path.dist,
      curve_path.radius);
}

std::vector<CurvePath> CalCurvePathConnectTwoPose(const Pos3d &start_pos,
                                                  const Pos3d &end_pos,
                                                  double r_min) {
  const double k_min_value = 1e-4;
  double rx_startpos_endpos = CalProjectInX(start_pos, end_pos);
  double ry_startpos_endpos = CalProjectInY(start_pos, end_pos);
  double ry_endpos_startpos = CalProjectInY(end_pos, start_pos);
  double theta = NormalizeAngle(end_pos.phi - start_pos.phi);

  std::vector<CurvePath> curve_paths;
  if (std::abs(rx_startpos_endpos) < k_min_value) {
    return curve_paths;
  }

  // a line path
  if (std::abs(ry_startpos_endpos) < k_min_value &&
      std::abs(theta) < k_min_value) {
    CurvePath line_path{start_pos.x, start_pos.y, start_pos.phi,
                        rx_startpos_endpos, 0};
    curve_paths.push_back(line_path);
    return curve_paths;
  }

  // a circle path
  if (std::abs(std::atan(ry_startpos_endpos / rx_startpos_endpos) -
               theta / 2.0) < k_min_value) {
    double r = rx_startpos_endpos / std::sin(theta);
    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath circle_path{start_pos.x, start_pos.y, start_pos.phi, r * theta,
                          r};
    curve_paths.push_back(circle_path);
    return curve_paths;
  }

  if (std::abs(1.0 - std::cos(theta)) <= k_min_value) {
    return curve_paths;
  }

  if (std::abs(ry_endpos_startpos) > std::abs(ry_startpos_endpos)) {
    double r = ry_startpos_endpos / (1.0 - std::cos(theta));
    double line_length =
        ry_endpos_startpos / std::sin(theta) - r * std::tan(theta / 2.0);

    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath line_path{start_pos.x, start_pos.y, start_pos.phi, line_length,
                        0};
    Pos3d min_pos = CalEndPosWithACurvePath(line_path);
    CurvePath circle_path{min_pos.x, min_pos.y, min_pos.phi, r * theta, r};
    curve_paths.push_back(line_path);
    curve_paths.push_back(circle_path);
  } else {
    double r = ry_endpos_startpos / (1.0 - std::cos(theta));
    double line_length =
        ry_startpos_endpos / std::sin(theta) - r * std::tan(theta / 2.0);

    if (std::abs(r) < r_min) {
      return curve_paths;
    }

    CurvePath circle_path{start_pos.x, start_pos.y, start_pos.phi, r * theta,
                          r};
    Pos3d min_pos = CalEndPosWithACurvePath(circle_path);
    CurvePath line_path{min_pos.x, min_pos.y, min_pos.phi, line_length, 0};
    curve_paths.push_back(circle_path);
    curve_paths.push_back(line_path);
  }

  return curve_paths;
}

std::vector<Pos3d> GetTrajFromCurvePathsConnect(const Pos3d &start_pos,
                                                const Pos3d &end_pos,
                                                double r_min,
                                                double resolution) {
  std::vector<Pos3d> traj_points;
  std::vector<CurvePath> curve_paths =
      CalCurvePathConnectTwoPose(start_pos, end_pos, r_min);
  if (curve_paths.empty()) {
    return traj_points;
  }

  for (const auto &path : curve_paths) {
    int traj_point_num = std::abs(path.dist) / resolution;
    for (int i = 0; i < traj_point_num; ++i) {
      const Pos3d path_start_pos{path.x, path.y, path.phi};
      Pos3d temp_pos = CalEndPosWithACurvePath(
          path_start_pos, sign(path.dist) * i * resolution, path.radius);
      traj_points.push_back(temp_pos);
    }
  }

  return traj_points;
}

}  // namespace math