#include "sisl_toolbox/circle.hpp"
#include "sisl.h"

Circle::Circle(double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint, int dimension, int order) 
    : Curve(dimension, order)
    , angle_{angle}
    , axis_{axis}
    , centrePoint_{centrePoint}
    {
        name_ = "Circle";

        // Generate a circle according to the parameters (angle, axis, startPoint, centrePoint).
        s1303(&startPoint[0], Epsge(), angle_, &centrePoint_[0], &axis_[0], Dimension(), &curve_, &statusFlag_);
        
        // Pick parameters range of the circle.
        s1363(curve_, &startParameter_s_, &endParameter_s_, &statusFlag_);

        // Pick curve length.
        s1240(curve_, Epsge(), &endParameter_m_, &statusFlag_);

        FromAbsSislToPos(startParameter_s_, startPoint_);
        FromAbsSislToPos(endParameter_s_, endPoint_);

        startParameter_m_ = startParameter_s_ * (endParameter_m_ / endParameter_s_);

        length_ = std::abs(endParameter_m_ - startParameter_m_);
    }

