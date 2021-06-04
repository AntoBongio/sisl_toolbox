#ifndef PATHMANAGER_HPP
#define PATHMANAGER_HPP

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include <ctrl_toolbox/HelperFunctions.h>

#include "sisl_toolbox/generic_curve.hpp"
#include "sisl_toolbox/straight_line.hpp"
#include "sisl_toolbox/circle.hpp"
#include "sisl_toolbox/defines.hpp" 


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

    std::shared_ptr<std::vector<Eigen::Vector3d>> Sampling(int samples) const;

    // Define [] operator
    std::shared_ptr<Curve>& operator[](std::size_t idx) { return curves_[idx]; }
    const std::shared_ptr<Curve>& operator[](std::size_t idx) const { return curves_[idx]; }

    auto LastCurve()  {return curves_[curvesNumber_ - 1];}

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

    /** 
     * @brief Convert an angle in degrees to [0, 360.0) interval
     * 
     * @param[in] angle The angle to be converted passed by copy
     * 
     * @return The along curve distance in meters from the stating point of the curve to abscissa parameter.
     */ 
    auto convertToAngleInterval (double angle) const {
        while(angle < 0) { angle += 360.0; }
        return std::fmod(angle, 360.0);
    };

    /** 
     * @brief Given the vertices of a polygon, compute the rectangle surrounding the figure (with 3 decimals precision).
     * 
     * @param[in] polygonVerteces const reference to the vector containing polygon's vertices
     * 
     * @return A tuple with: (maxX, minX, maxY, minY)
     */ 
    std::tuple<double, double, double, double> evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces) const;

    /** 
     * @brief Compute distance between two points
     * 
     * @param[in] vec1 First point expresed as Eigen::Vector3d
     * @param[in] vec2 Second point expresed as Eigen::Vector3d
     * 
     * @return Distance between two points
     */ 
    auto Distance (Eigen::Vector3d const& vec1, Eigen::Vector3d const& vec2) {
        return std::sqrt(std::pow(vec1[0] - vec2[0], 2) + std::pow(vec1[1] - vec2[1], 2) + std::pow(vec1[2] - vec2[2], 2));
    };

    std::vector<std::shared_ptr<Curve>> curves_;
    int curvesNumber_;
    double length_;
    double currentAbscissa_;
    int currentCurveId_;

};

#endif
