#include "sisl_toolbox/Circle.hpp"
#include "sisl.h"
// (2, 3, 3, 6.20, axis, middlePoint, centreCircle));
Circle::Circle(int type, int dimension, int order, double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint) 
    : Curve(dimension, order, type)
    , angle_{angle}
    , axis_{axis}
    , centrePoint_{centrePoint}
    {
        
        // Generate a circle according to the parameters (angle, axis, startPoint, centrePoint).
        s1303(&startPoint_[0], Epsge(), angle_, &centrePoint_[0], &axis_[0], Dimension(), &curve_, &statusFlag_);

        // Pick parameters range of the circle.
        s1363(curve_, &startParameter_, &endParameter_, &statusFlag_);

        // Pick curve length.
        s1240(curve_, Epsge(), &length_, &statusFlag_);

        FromAbsToPos(startParameter_, startPoint_);
        FromAbsToPos(endParameter_, endPoint_);
    }

