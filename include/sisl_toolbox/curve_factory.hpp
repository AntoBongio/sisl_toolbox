#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sisl_toolbox/defines.hpp>

struct SISLCurve; /** Forward declaration */
class Curve; /** Forward declaration */
class Circle; /** Forward declaration */
class StraightLine; /** Forward declaration */
class GenericCurve; /** Forward declaration */

class CurveFactory
{
public:

    /** PROBLEMA: in questo modo vi è conflitto tra i costruttori */
    template <typename T, typename... Args>
    static std::shared_ptr<T> NewCurve(int type, Args... args) {

        std::shared_ptr<T> curve;

        switch(type) {
            case 0:
                std::cout << "[CurveFactory] -> Building a GenericCurve" << std::endl;
                return std::make_shared<T>(args...);
                break;
            case 1:
            {
                std::cout << "[CurveFactory] -> Building a StraightLine" << std::endl;
                return std::make_shared<T>(args...);
                break;
            } 
            case 2:
                std::cout << "[CurveFactory] -> Building a Circle" << std::endl; 
                return std::make_shared<T>(args...);
                break;
        }
    }

    // static std::shared_ptr<Curve> NewCurve(Parameters params);

    // static std::shared_ptr<StraightLine> NewStraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint);
    // static std::shared_ptr<Circle> NewCircle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint);
    // static std::shared_ptr<GenericCurve> NewGenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, 
    //     std::vector<double> weights, std::vector<double> coefficients = {});


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

