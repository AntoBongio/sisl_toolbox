#pragma once

#include "sisl_toolbox/curve.hpp"

/**
 * @class CircularArc
 *
 * @brief Class derived from Curve. It adds a specific constructor for Circular Arc Curve objects as well as the specific getters. 
 */
class CircularArc : public Curve{

public:

    /** 
     * @brief Circular Arc constructor based on s1303() SISL routine.
     * 
     * @param angle The rotational angle (in rad). Counterclockwise around axis. If the rotational angle is outside <−2π, +2π> then a closed curve is produced.
     * @param axis Normal vector to plane in which the circle lies.
     * @param startPoint Start point of the circular arc.
     * @param centrePoint Centre point of the circular arc.
     * 
     * @param dimension Parameter used in Curve constructor -> default = 3
     * @param order Parameter used in Curve constructor -> default = 3
     */
    CircularArc(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint, int dimension = 3, int order = 3);
    

    // Getter / Setter methods
    auto Angle() const& {return angle_;}
    auto Axis() const& {return axis_;}
    auto CentrePoint() const& {return centrePoint_;}

private:

    double angle_;
    Eigen::Vector3d axis_;
    Eigen::Vector3d centrePoint_;
};
