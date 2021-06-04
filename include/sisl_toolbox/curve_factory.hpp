#pragma once

#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

struct SISLCurve; /** Forward declaration */
class Curve; /** Forward declaration */

class CurveFactory
{
public:

    /*
    static std::shared_ptr<Curve> NewGenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights);
    static std::shared_ptr<Curve> NewStraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint);
    static std::shared_ptr<Curve> NewCircle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint);

    static std::shared_ptr<Curve> NewGenericCurve(SISLCurve * curve);
    static std::shared_ptr<Curve> NewStraightLine(SISLCurve * curve);
    static std::shared_ptr<Curve> NewCircle(SISLCurve * curve);
    */

private:

};

