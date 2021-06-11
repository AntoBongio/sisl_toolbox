#pragma once

#include "sisl_toolbox/curve.hpp"

class Circle : public Curve{

public:

    /** 
     * @brief Circle constructor based on s1303() SISL routine.
     * 
     * @param angle The rotational angle. Counterclockwise around axis. If the rotational angle is outside <−2π, +2π> then a closed curve is produced.
     * @param axis Normal vector to plane in which the circle lies. Used if dim = 3.
     * @param startPoint Start point of the circular arc
     * @param centrePoint Point on the axis of the circle.
     * 
     * @param dimension Parameter used in Curve constructor -> default = 3
     * @param order Parameter used in Curve constructor -> default = 3
     */
    Circle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint, int dimension = 3, int order = 3);
    

    // Getter / Setter methods
    auto Angle() const& {return angle_;}
    auto Axis() const& {return axis_;}
    auto CentrePoint() const& {return centrePoint_;}

private:

    double angle_;
    Eigen::Vector3d axis_;
    Eigen::Vector3d centrePoint_;


};
