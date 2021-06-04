#include "sisl_toolbox/curve_factory.hpp"
#include "sisl_toolbox/curve.hpp"
#include "sisl.h"











/*
std::shared_ptr<Curve> CurveFactory::NewStraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint) 
    {        
        if(startPoint == endPoint) {
            auto curveObj = Curve(1);
            curveObj.Length(0);
            curveObj.StartParameter(0);
            curveObj.EndParameter(0);
            curveObj.StartPoint() = startPoint;
            curveObj.EndPoint() = endPoint;

            return std::make_shared<Curve>(curveObj);
        }
        else {

            auto curveObj = Curve(startPoint, endPoint);

            return std::make_shared<Curve>(curveObj);
        }

    }

std::shared_ptr<Curve> CurveFactory::NewCircle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint) {

    if(startPoint == centrePoint) {
            auto curveObj = Curve(2);
            curveObj.Length(0);
            curveObj.StartParameter(0);
            curveObj.EndParameter(0);
            curveObj.StartPoint() = startPoint;
            curveObj.EndPoint() = startPoint;

            return std::make_shared<Curve>(curveObj);
        }
        else {

            auto curveObj = Curve(angle, axis, startPoint, centrePoint);

            return std::make_shared<Curve>(curveObj);
        }
}

std::shared_ptr<Curve> CurveFactory::NewGenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, 
    std::vector<double> weights) {

        auto curveObj = Curve(degree, knots, points, weights);

        return std::make_shared<Curve>(curveObj);
}


std::shared_ptr<Curve> CurveFactory::NewGenericCurve(SISLCurve * curve) {

    auto curveObj = Curve(0, curve);

    return std::make_shared<Curve>(curveObj);
}

std::shared_ptr<Curve> CurveFactory::NewStraightLine(SISLCurve * curve) {

    auto curveObj = Curve(1, curve);

    return std::make_shared<Curve>(curveObj);
}

std::shared_ptr<Curve> CurveFactory::NewCircle(SISLCurve * curve) {

    auto curveObj = Curve(2, curve);

    return std::make_shared<Curve>(curveObj);
}
*/
