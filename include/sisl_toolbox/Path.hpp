#ifndef PATHMANAGER_HPP
#define PATHMANAGER_HPP

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "sisl_toolbox/GenericCurve.hpp"
#include "sisl_toolbox/StraightLine.hpp"
#include "sisl_toolbox/Circle.hpp"
#include "sisl_toolbox/Defines.hpp" 


/**
 * @class Path
 *
 * @brief 
 *
 */
class Path {

public:
    Path();

    Path(std::vector<Parameters>& parameters);

    template <typename T>
    void AddCurveBack(std::shared_ptr<T> curve);

    void SavePath(int samples, std::string path);

    /**
     * @brief Reverse the whole path
     */ 
    void Reverse();

    Eigen::Vector3d FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa);

    void ExtractSection(double offset, double abscissa, int curveId, std::shared_ptr<Path>& pathPortion);

    void ExtractSection(double offset, std::shared_ptr<Path>& pathPortion);

    void MoveCurrentState(double offset, Eigen::Vector3d& point);

    void MoveState(double offset, double& abscissa, int& curveId, Eigen::Vector3d& point);

    double AlongPathDistance();

    // Getter / Setter
    auto Curves() {return curves_;}
    auto CurvesNumber() {return curvesNumber_;}
    auto Length() {return length_;}
    
    // Da togliere
    auto CurrentAbscissa() {return currentAbscissa_;}
    auto CurrentCurveId() {return currentCurveId_;}

private:
    std::vector<std::shared_ptr<Curve>> curves_;
    int curvesNumber_;
    double length_;
    double currentAbscissa_;
    int currentCurveId_;

};
#endif
