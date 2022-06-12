#ifndef MATH_UTILS
#define MATH_UTILS

#include "basic_type.h"
#include <utility>

#define M_PI		3.14159265358979323846

namespace math {
    double NormalizeAngle(const double angle);
    std::pair<double, double> Cartesian2Polar(double x, double y);
    double DistanceTo(const LineSegment2d &line_segment2d, const Vec2d &point);
    double GetVecAngle(Vec2d v);
    bool HasOverlap(Box2d bounding_box, LineSegment2d linesegment);

}// namespace math



#endif 
