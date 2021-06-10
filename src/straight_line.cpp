#include "sisl_toolbox/straight_line.hpp"
#include "sisl.h"


StraightLine::StraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, int dimension, int order)
    : Curve(dimension, order, STRAIGHTLINE)
    {        
        name_ = "Straight Line";

        if(startPoint == endPoint) {
            startParameter_s_ = 0;
            endParameter_s_ = 0;
            startParameter_m_ = 0; 
            endParameter_m_ = 0;
            curve_ = nullptr;
        }
        else {
            // Generate a straight line from startPoint to endPoint
            s1602(&startPoint[0], &endPoint[0], Order(), Dimension(), startParameter_s_, &endParameter_s_, &curve_, &statusFlag_);

            // Pick parameters range of the line.
            s1363(curve_, &startParameter_s_, &endParameter_s_, &statusFlag_);

            // Pick curve length
            s1240(curve_, Epsge(), &endParameter_m_, &statusFlag_);

            FromAbsSislToPos(startParameter_s_, startPoint_);
            FromAbsSislToPos(endParameter_s_, endPoint_); 

            startParameter_m_ = startParameter_s_ * (endParameter_m_ / endParameter_s_); 
        }
    }