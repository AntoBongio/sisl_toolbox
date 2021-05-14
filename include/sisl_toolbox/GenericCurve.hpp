#ifndef GENERICCURVE_HPP
#define GENERICCURVE_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sisl_toolbox/Curve.hpp"

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

    // Getter / Setter methods
    auto Degree() {return degree_;}
    auto Knots() {return knots_;}
    auto Points() {return points_;}
    auto Weights() {return weights_;}
    auto Coefficients() {return coefficients_;}

private:
    int degree_;
    std::vector<double> knots_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<double> weights_;
    std::vector<double> coefficients_;
};
#endif