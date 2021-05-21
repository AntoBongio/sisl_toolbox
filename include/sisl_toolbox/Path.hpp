#ifndef PATHMANAGER_HPP
#define PATHMANAGER_HPP

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include <ctrl_toolbox/HelperFunctions.h>

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

    Path(std::vector<Parameters> & parameters);

    Path(std::vector<Eigen::Vector3d> & points);

    Path(double angle, double offset, std::vector<Eigen::Vector3d>& polygonVerteces);

    template <typename T>
    void AddCurveBack(std::shared_ptr<T> curve);

    void SavePath(int samples, std::string const path) const;

    /**
     * @brief Reverse the whole path
     */ 
    void Reverse();

    Eigen::Vector3d FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa);

    void ExtractSection(double offset, double abscissa, int curveId, std::shared_ptr<Path>& pathPortion);

    void ExtractSection(double offset, std::shared_ptr<Path>& pathPortion);

    void MoveCurrentState(double offset, Eigen::Vector3d& point);

    void MoveState(double offset, double& abscissa, int& curveId, Eigen::Vector3d& point);

    double AlongPathDistance() const;

    std::vector<Eigen::Vector3d> Intersection(std::shared_ptr<Path> otherPath);

    std::vector<Eigen::Vector3d> Intersection(int curveId, std::shared_ptr<Path> otherPath);

    

    // Getter / Setter
    auto Curves() const& {return curves_;}
    auto Curves() & {return curves_;}
    auto Curves() && {return std::move(curves_);}

    auto CurvesNumber() const& {return curvesNumber_;}
    auto CurvesNumber() & {return curvesNumber_;}
    auto CurvesNumber() && {return std::move(curvesNumber_);}

    auto Length() const& {return length_;}
    auto Length() & {return length_;}
    auto Length() && {return std::move(length_);}
    
    // Da togliere
    auto CurrentAbscissa() const& {return currentAbscissa_;}
    auto CurrentAbscissa() & {return currentAbscissa_;}
    auto CurrentAbscissa() && {return std::move(currentAbscissa_);}

    auto CurrentCurveId() const& {return currentCurveId_;}
    auto CurrentCurveId() & {return currentCurveId_;}
    auto CurrentCurveId() && {return std::move(currentCurveId_);}

private:
    std::vector<std::shared_ptr<Curve>> curves_;
    int curvesNumber_;
    double length_;
    double currentAbscissa_;
    int currentCurveId_;

};
#endif
