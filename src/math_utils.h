#ifndef MATH_UTILS
#define MATH_UTILS

#include <memory>
#include <utility>
#include <vector>

#include "basic_type.h"

#define M_PI 3.14159265358979323846

constexpr double kMathEpsilon = 1e-10;

namespace math {
double sign(double x);
double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y, double length);
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
double CalProjectInX(const Pos3d &pos, const Vec2d &point);
double CalProjectInX(const Pos3d &pos1, const Pos3d &pos2);
double CalProjectInY(const Pos3d &pos, const Vec2d &point);
double CalProjectInY(const Pos3d &pos1, const Pos3d &pos2);
void ConnectByLineCircle(Pos3d current_pos, Pos3d end_pos);
Pos3d CalEndPosWithACurvePath(const Pos3d &start_pos, double dist,
                              double radius);
Pos3d CalEndPosWithACurvePath(CurvePath curve_path);
std::vector<CurvePath> CalCurvePathConnectTwoPose(const Pos3d &start_pos,
                                                  const Pos3d &end_pos,
                                                  double r_min);
std::vector<Pos3d> GetTrajFromCurvePathsConnect(const Pos3d &start_pos,
                                                const Pos3d &end_pos,
                                                double r_min,
                                                double resolution);

}  // namespace math

#endif
