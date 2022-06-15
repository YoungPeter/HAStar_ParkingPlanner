#ifndef MATH_UTILS
#define MATH_UTILS

#include "basic_type.h"
#include <utility>

#define M_PI 3.14159265358979323846

constexpr double kMathEpsilon = 1e-10;

namespace math
{
    double PtSegDistance(double query_x, double query_y, double start_x,
                         double start_y, double end_x, double end_y,
                         double length);
    double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                     const Vec2d &end_point_2);
    double NormalizeAngle(const double angle);
    std::pair<double, double> Cartesian2Polar(double x, double y);
    double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point);
    double GetVecAngle(Vec2d v);
    double DistanceTo(const Box2d &bounding_box, const Vec2d &point);
    double DistanceTo(const Box2d &bounding_box, const LineSegment2d &line_segment);
    bool IsPointIn(const Box2d &bounding_box, const Vec2d &point);
    bool HasOverlap(const Box2d &bounding_box, const LineSegment2d &line_segment);

} // namespace math

#endif
