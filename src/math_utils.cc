#include "math_utils.h"

#include <cmath>



namespace math
{
double NormalizeAngle(const double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0){
   a += (2.0 * M_PI);
  }
  return a - M_PI;
}


std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}


double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point)  {
  if (line_segment2d.length_ <= 1e-10) {
    return std::hypot(point.x_ - line_segment2d.start_.x_, point.y_ - line_segment2d.start_.y_);  //point.DistanceTo(start_);
  }
  const double x0 = point.x_ - line_segment2d.start_.x_ ;
  const double y0 = point.y_  - line_segment2d.start_.y_ ;
  const double proj = x0 * line_segment2d.unit_direction_.x_  + y0 * line_segment2d.unit_direction_.y_ ;
  if (proj <= 0.0) {
    return std::hypot(x0, y0);
  }
  if (proj >= line_segment2d.length_) {
    return std::hypot(point.x_ - line_segment2d.end_.x_, point.y_ - line_segment2d.end_.y_);
  }
  return std::abs(x0 * line_segment2d.unit_direction_.y_  - y0 * line_segment2d.unit_direction_.x_);
}

double GetVecAngle(Vec2d v)  {
   return std::atan2(v.y_, v.x_);
}

bool HasOverlap(Box2d bounding_box, LineSegment2d linesegment){
  return false;
  // if (line_segment.length() <= 1e-10) {
  //   return IsPointIn(line_segment.start());
  // }
  // if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x() ||
  //     std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x() ||
  //     std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y() ||
  //     std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y()) {
  //   return false;
  // }
  // return DistanceTo(line_segment) <= 1e-10;

}

} // namespace math