#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <map>
#include <eigen3/Eigen/Dense>

#include "sisl_toolbox/generic_curve.hpp"
#include "sisl_toolbox/straight_line.hpp"
#include "sisl_toolbox/circle.hpp"
#include "sisl_toolbox/defines.hpp" 

class PathFactory;

/** TODO: Fare forward declaration delle classi ed includerle nel .cpp ? */

/**
 * @class Path
 *
 * @brief 
 *
 */
class Path {

public:
    Path();

    /**
     * @brief Add a curve back
     * 
     * @param curve std::shared_ptr<T> with T in (StraightLine, Circle, GenericCurve). The curve to be added.
     */
    template <typename T>
    void AddCurveBack(std::shared_ptr<T> curve) {
        curves_.push_back(curve);
        endParameter_m_ += curve->Length();
        ++curvesNumber_; 
    }


    /**
     * @brief Reverse the whole path
     */
    void Reverse();

    
    /**
     * @brief Find Closest Point w.r.t. the path
     * 
     * @param[in] worldF_position point in the find closest point problem.
     * @param[out] curveId Id of the curve containing the closest point.
     * @param[out] abscissa_m Abscissa (in meters) of the closest point on the path, it is the abscissa_m of the curve identified with curveId.
     * 
     * @return An Eigen::Vector3d representing the closest point.
     */
    Eigen::Vector3d FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa_m);


    /**
     * @brief 
     * 
     * @param samples
     * 
     * @return std::shared_ptr<std::vector<Eigen::Vector3d>> containing the points.
     */
    std::shared_ptr<std::vector<Eigen::Vector3d>> Sampling(int samples) const;


    /**
     * @brief Return last curve added to the path
     *  
     * @return curves_[curvesNumber_ - 1].
     */
    auto LastCurve()  {return curves_[curvesNumber_ - 1];}


    /**
     * @brief Convert from Abscissa path parameter to Abscissa curve parameter. If the abscissa_m is beyond of before the 
     * path parametrization extrema, the correct path extreme is returned and the overBound struct is filled.
     * 
     * @param[in] abscissa_m Path abscissa value.
     *  
     * @return A tuple containing respectively: abscissa of the curve, the curveId, and the overBound struct.
     */
    std::tuple<double, int, overBound> PathAbsToCurveAbs(double abscissa_m);


    /**
     * @brief Convert from Abscissa curve parameter to Abscissa path parameter. If the abscissaCurve_m is beyond of before the 
     * curve parametrization extrema, the correct curve extreme is returned and the overBound struct is filled.
     * 
     * @param[in] abscissaCurve_m Curve abscissa value.
     * @param[in] curveId Identifier for the curve.
     *  
     * @return A tuple containing respectively: abscissa of the path, and the overBound struct.
     */
    std::tuple<double, overBound> CurveAbsToPathAbs(double abscissaCurve_m, int curveId);


    /**
     * @brief Given an abscissa in meters return the corresponding point on path.
     * 
     * @param[in] abscissa_m abscissa on the path in meters.
     *  
     * @return Eigen::Vector3d containing the point at abscissa_m.
     */
    Eigen::Vector3d At(double abscissa_m);


    /**
     * @brief Extract a path portion given as input the start/end values.
     * 
     * @param[in] startValue_m Start abscissa value for the path extraction.
     * @param[in] endValue_m End abscissa value for the path extraction.
     *  
     * @return std::shared_ptr<Path> contaning the path portion.
     */
    std::shared_ptr<Path> ExtractSection(double startValue_m, double endValue_m);


    /**
     * @brief Eval intersections among two path.
     * 
     * @param[in] otherPath shared_ptr to the other path.
     *  
     * @return std::vector<Eigen::Vector3d> contaning the intersection points.
     */
    std::vector<Eigen::Vector3d> Intersection(std::shared_ptr<Path> otherPath);

    /**
     * @brief Eval intersections among the i-th curve of the current path and another path.
     * 
     * @param[in] curveId Id of the curve of the current path.
     * @param[in] otherPath shared_ptr to the other path.
     *  
     * @return std::vector<Eigen::Vector3d> contaning the intersection points.
     */
    std::vector<Eigen::Vector3d> Intersection(int curveId, std::shared_ptr<Path> otherPath);  

    // Define [] operator
    std::shared_ptr<Curve>& operator[](std::size_t idx) { return curves_[idx]; }
    const std::shared_ptr<Curve>& operator[](std::size_t idx) const { return curves_[idx]; }

    friend std::ostream& operator<< (std::ostream& os, const Path& obj) {
        return os 
            << "Path name: " << obj.name_
            << " | Length: " << (obj.endParameter_m_ - obj.startParameter_m_)
            << " | Parametrization interval: [" << obj.startParameter_m_ << ", " << obj.endParameter_m_ << "]"
            << " | Curves contained: " << obj.curvesNumber_;
    };

    // Getters
    auto Curves() const& {return curves_;}
    auto CurvesNumber() const& {return curvesNumber_;}
    auto Length() const& {return endParameter_m_;}
    auto StartParameter() const& {return startParameter_m_;}
    auto EndParameter() const& {return endParameter_m_;}
    auto Name() const& {return name_;}


private:

    friend PathFactory;

    std::vector<std::shared_ptr<Curve>> curves_;
    int curvesNumber_;
    double startParameter_m_;
    double endParameter_m_;
    std::string name_{};

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

};

