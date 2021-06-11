#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sisl_toolbox/curve.hpp"


class GenericCurve : public Curve{

public:

    /** 
     * @brief GenericCurve constructor based on newCurve() SISL routine.
     * 
     * @param degree
     * @param knots
     * @param points
     * @param weights
     * @param coefficients
     * 
     * @param dimension Parameter used in Curve constructor -> default = 3
     * @param order Parameter used in Curve constructor -> default = 3
     */ 
    GenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights, 
        std::vector<double> coefficients, int dimension = 3, int order = 3);

    // Getters
    auto Degree() const& {return degree_;}
    auto Knots() const& {return knots_;}
    auto Points() const& {return points_;}
    auto Weights() const& {return weights_;}
    auto Coefficients() const& {return coefficients_;}

private:
    int degree_;
    std::vector<double> knots_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<double> weights_;
    std::vector<double> coefficients_;
};