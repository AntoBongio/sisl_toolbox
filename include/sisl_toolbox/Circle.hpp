#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "sisl_toolbox/Curve.hpp"

class Circle : public Curve{

public:
    /** 
     * @brief Circle constructor based on s1303() SISL routine.
     * 
     * @param type Curve type -> 0 : Generic Curve ; 1 : Straight Line ; 2 : Circle
     * @param dimension Parameter used in Curve constructor
     * @param order Parameter used in Curve constructor
     * 
     * @param angle The rotational angle. Counterclockwise around axis. If the rotational angle is outside <−2π, +2π> then a closed curve is produced.
     * @param axis Normal vector to plane in which the circle lies. Used if dim = 3.
     * @param startPoint Start point of the circular arc
     * @param centrePoint Point on the axis of the circle.
     */ 
    Circle(int type, int dimension, int order, double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint);

    // Getter / Setter methods
    auto Angle() const& {return angle_;}
    auto Angle() & {return angle_;}
    auto Angle() && {return std::move(angle_);}

    auto Axis() const& {return axis_;}
    auto Axis() & {return axis_;}
    auto Axis() && {return std::move(axis_);}

    auto CentrePoint() const& {return centrePoint_;}
    auto CentrePoint() & {return centrePoint_;}
    auto CentrePoint() && {return std::move(centrePoint_);}

private:
    double angle_;
    Eigen::Vector3d axis_;
    Eigen::Vector3d centrePoint_;
};
#endif
