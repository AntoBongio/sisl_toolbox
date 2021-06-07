#include "sisl_toolbox/curve_factory.hpp"
#include "sisl_toolbox/curve.hpp"
#include "sisl_toolbox/straight_line.hpp"
#include "sisl_toolbox/circle.hpp"
#include "sisl_toolbox/generic_curve.hpp"
#include "sisl.h"


// // Al momento non funziona
// template <typename... Args>
// std::shared_ptr<Curve> CurveFactory::NewCurve(int type, Args... args) {

//     std::shared_ptr<Curve> curve;

//     switch(type) {
//         case 0:
//             //curve = std::make_shared<GenericCurve>(args...);
//             break;
//         case 1:
//             //curve = std::make_shared<StraightLine>(args...);
//             break;
//         case 2: 
//             //curve = std::make_shared<Circle>(args...);
//             break;
//     }
//     return curve;
// }


// std::shared_ptr<Curve> CurveFactory::NewCurve(Parameters params) {

//     std::shared_ptr<Curve> curvePtr;

//     switch(params.type) {
//         case 0:
//             {
//                 std::cout << "[CurveFactory] -> Building a GenericCurve" << std::endl;
//                 //curvePtr = std::make_shared<GenericCurve>(params);
//                 break;
//             }
//         case 1:
//             {
//                 std::cout << "[CurveFactory] -> Building a StraightLine" << std::endl;
//                 auto curve = StraightLine(params);
//                 //curvePtr = std::make_shared<StraightLine>(curve);
//                 break;
//             }
//         case 2: 
//             {
//                 std::cout << "[CurveFactory] -> Building a Circle" << std::endl;
//                 //curvePtr = std::make_shared<Circle>(params);
//                 break;
//             }
//     }
//     return curvePtr;
// }

// std::shared_ptr<StraightLine> CurveFactory::NewStraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint) {
//     return std::make_shared<StraightLine>(startPoint, endPoint);
// }


// std::shared_ptr<Circle> CurveFactory::NewCircle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint) {
//     return std::make_shared<Circle>(angle, axis, startPoint, centrePoint);
// }

// std::shared_ptr<GenericCurve> CurveFactory::NewGenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights, 
//     std::vector<double> coefficients) {
//     return std::make_shared<GenericCurve>(degree, knots, points, weights, coefficients);
// }










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
