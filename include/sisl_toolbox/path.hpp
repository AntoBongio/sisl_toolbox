#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <map>
#include <eigen3/Eigen/Dense>

class PathFactory;
class Curve;

/**
 * @class Path
 *
 * @brief This class is used to build a complex path starting from the Curve objects. The path is parametrized in meters 
 *        with abscissa in the interval [0, pathLength].
 */
class Path {

public:
    Path();

    /**
     * @brief Add a curve back
     * 
     * @param curve std::shared_ptr<T> with T in (StraightLine, CircularArc, GenericCurve). The curve to be added.
     */
    template <typename T>
    void AddCurveBack(std::shared_ptr<T> curve) {
        curves_.push_back(curve);
        endParameter_m_ += curve->Length();
        length_ = endParameter_m_ - startParameter_m_;
        ++curvesNumber_; 
    }

    /**
     * @brief Convert from Abscissa path parameter to Abscissa curve parameter. If the abscissa_m is beyond of before the 
     * path parametrization extrema.
     * 
     * @param[in] abscissa_m Path abscissa value.
     *  
     * @return A tuple containing respectively: abscissa of the curve, curve Id.
     */
    std::tuple<double, int> PathAbsToCurveAbs(double abscissa_m);

    /**
     * @brief Convert from Abscissa curve parameter to Abscissa path parameter. If the abscissaCurve_m is beyond or before the 
     * curve parametrization extrema, an exception is thrown.
     * 
     * @param[in] abscissaCurve_m Curve abscissa value.
     * @param[in] curveId Identifier for the curve.
     *  
     * @return The abscissa of the path.
     */
    double CurveAbsToPathAbs(double abscissaCurve_m, int curveId);

    /**
     * @brief Given an abscissa return the corresponding point on path.
     * 
     * @param[in] abscissa_m abscissa on the path (in meters).
     *  
     * @return Eigen::Vector3d containing the point at abscissa_m.
     */
    Eigen::Vector3d At(double abscissa_m);

    /**
     * @brief Given an abscissa in meters return the derivatives up to the n-th one at abscissa_m point.
     * 
     * @param[in] order evaluate the derivatives from 1 up to order.
     * @param[in] abscissa_m abscissa on the curve in meters.
     *  
     * @return std::vector<Eigen::Vector3d> containing the point at abscissa_m.
     */
    std::vector<Eigen::Vector3d> Derivate(int order, double abscissa_m);

    /**
     * @brief Given an abscissa in meters return the curvature at abscissa_m point.
     * 
     * @param[in] abscissa_m abscissa on the curve in meters.
     *  
     * @return curvature value.
     */
    double Curvature(double abscissa_m);

    /**
     * @brief Sampling the path. The total points are equally distributed among the curves, without keeping into account 
     *        the length of each curve.
     * 
     * @param samples
     * 
     * @return std::shared_ptr<std::vector<Eigen::Vector3d>> containing the points.
     */
    std::shared_ptr<std::vector<Eigen::Vector3d>> Sampling(int samples) const;

    /**
     * @brief Reverse the whole path
     */
    void Reverse();
    
    /**
     * @brief Find Closest Point w.r.t. the path.
     * 
     * @param[in] worldF_position point in the find closest point problem.
     * @param[out] curveId Id of the curve containing the closest point.
     * @param[out] abscissa_m Abscissa (in meters) of the closest point on the path, it is the abscissa_m of the curve identified with curveId.
     * 
     * @return An Eigen::Vector3d representing the closest point.
     */
    Eigen::Vector3d FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa_m);    

    /**
     * @brief Find Closest Point w.r.t. the path. This overloaded version does not give back, filing the arguments,
     *        the abscissa_m and curveID of the curve where the point lies.  
     * 
     * @param[in] worldF_position point in the find closest point problem.
     *  
     * @return An Eigen::Vector3d representing the closest point.
     */
    Eigen::Vector3d FindClosestPoint(Eigen::Vector3d& worldF_position);  

    /**
     * @brief Find Abscissa of the Closest Point w.r.t. the path.  
     * 
     * @param[in] worldF_position point in the find closest point problem.
     *  
     * @return The abscissa of the closest point on path.
     */
    double FindAbscissaClosestPoint(Eigen::Vector3d& worldF_position);

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

    /**
     * @brief Eval intersections among curve passed as argument and the current path.
     * 
     * @param[in] otherCurve shared_ptr to the curve.
     *  
     * @return std::vector<Eigen::Vector3d> contaning the intersection points.
     */
    std::vector<Eigen::Vector3d> Intersection(std::shared_ptr<Curve> otherCurve);

    /** TODO: Aggiustare!!! */
    /**
    * @brief Eval e tangent frame at the abscissa.
    * 
    * @param[in] abscissa Parameter value where to calculate the tangent frame.
    * @param[out] tangent Tangent component of the tangent 3D frame.
    * @param[out] normal Normal component of the tangent 3D frame.
    * @param[out] binormal Binormal component of the tangent 3D frame.
    */
    void EvalTangentFrame(double abscissa_m, Eigen::Vector3d& tangent, Eigen::Vector3d& normal, Eigen::Vector3d& binormal);

    // Define [] operator
    std::shared_ptr<Curve>& operator[](std::size_t const idx) { return curves_[idx]; }
    const std::shared_ptr<Curve>& operator[](std::size_t const idx) const { return curves_[idx]; }


    friend std::ostream& operator<< (std::ostream& os, const Path& obj) {
        return os 
            << "Path name: " << obj.name_
            << " | Length: " << (obj.length_)
            << " | Parametrization interval: [" << obj.startParameter_m_ << ", " << obj.endParameter_m_ << "]"
            << " | Curves contained: " << obj.curvesNumber_;
    };


    /**
     * @brief Return last curve added to the path
     *  
     * @return curves_[curvesNumber_ - 1].
     */
    auto LastCurve()  {return curves_[curvesNumber_ - 1];}



    // Getters
    auto Curves() const& {return curves_;}
    auto CurvesNumber() const& {return curvesNumber_;}
    auto Length() const& {return length_;}
    auto StartParameter() const& {return startParameter_m_;}
    auto EndParameter() const& {return endParameter_m_;}
    auto Name() const& {return name_;}


private:

    friend PathFactory;

    std::vector<std::shared_ptr<Curve>> curves_;
    int curvesNumber_;
    double length_;
    double startParameter_m_;
    double endParameter_m_;
    std::string name_{};


};

