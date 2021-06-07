#ifndef GENERICCURVE_HPP
#define GENERICCURVE_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sisl_toolbox/curve.hpp"

class GenericCurve : public Curve{

public:
    /** 
     * @brief GenericCurve constructor based on newCurve() SISL routine.
     * 
     * @param type Curve type -> 0 : Generic Curve ; 1 : Straight Line ; 2 : Circle
     * @param dimension Parameter used in Curve constructor
     * @param order Parameter used in Curve constructor
     * 
     * @param degree
     * @param knots
     * @param points
     * @param weights
     * @param coefficients
     */ 
    GenericCurve(int type, int dimension, int order, int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, 
        std::vector<double> weights, std::vector<double> coefficients);

    GenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights, 
        std::vector<double> coefficients, int type = 0, int dimension = 3, int order = 3);

    // Getter / Setter methods
    auto Degree() const& {return degree_;}
    auto Degree() & {return degree_;}
    auto Degree() && {return std::move(degree_);}

    auto Knots() const& {return knots_;}
    auto Knots() & {return knots_;}
    auto Knots() && {return std::move(knots_);}

    auto Points() const& {return points_;}
    auto Points() & {return points_;}
    auto Points() && {return std::move(points_);}

    auto Weights() const& {return weights_;}
    auto Weights() & {return weights_;}
    auto Weights() && {return std::move(weights_);}

    auto Coefficients() const& {return coefficients_;}
    auto Coefficients() & {return coefficients_;}
    auto Coefficients() && {return std::move(coefficients_);}

private:
    int degree_;
    std::vector<double> knots_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<double> weights_;
    std::vector<double> coefficients_;
};
#endif