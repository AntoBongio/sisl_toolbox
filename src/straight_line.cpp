#include "sisl_toolbox/straight_line.hpp"
#include "sisl.h"

StraightLine::StraightLine(int type, int dimension, int order, Eigen::Vector3d startPoint, Eigen::Vector3d endPoint) 
    : Curve(dimension, order, type)
    {        

        if(startPoint == endPoint) {
            length_ = 0;
            startParameter_ = -1;
            endParameter_ = -1;
        }
        else {
           // Generate a straight line from startPoint to endPoint
            s1602(&startPoint[0], &endPoint[0], Order(), Dimension(), startParameter_, &endParameter_, &curve_, &statusFlag_);

            // Pick parameters range of the line.
            s1363(curve_, &startParameter_, &endParameter_, &statusFlag_);

            // Pick curve length
            s1240(curve_, Epsge(), &length_, &statusFlag_);

            FromAbsToPos(startParameter_, startPoint_);
            FromAbsToPos(endParameter_, endPoint_); 
        }

        

        /*
        std::cout << "[StraightLine constructor] -> (startPoint_, startParameter_): ([" << startPoint_[0] << ", " << startPoint_[1] << ", " 
            << startPoint_[2]  << "], " << startParameter_ << ") / (endParameter_, endPoint_): ([" << endPoint_[0] << ", " 
            << endPoint_[1] << ", " << endPoint_[2] << "], " << endParameter_ << ")" << std::endl;
        */
    }

StraightLine::StraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, int dimension, int order)
    : Curve(dimension, order, STRAIGHTLINE)
    {        

        if(startPoint == endPoint) {
            length_ = 0;
            startParameter_ = 0;
            endParameter_ = 0;
            curve_ = nullptr;
        }
        else {
            // Generate a straight line from startPoint to endPoint
            s1602(&startPoint[0], &endPoint[0], Order(), Dimension(), startParameter_, &endParameter_, &curve_, &statusFlag_);

            // Pick parameters range of the line.
            s1363(curve_, &startParameter_, &endParameter_, &statusFlag_);

            // Pick curve length
            s1240(curve_, Epsge(), &length_, &statusFlag_);

            FromAbsToPos(startParameter_, startPoint_);
            FromAbsToPos(endParameter_, endPoint_); 
        }
    }

// Test con Parameters struct
// StraightLine::StraightLine(Parameters params, int type, int dimension, int order)
//     : Curve(dimension, order, type)
//     {        

//         if(params.startPoint == params.endPoint) {
//             length_ = 0;
//             startParameter_ = 0;
//             endParameter_ = 0;
//         }
//         else {
//            // Generate a straight line from startPoint to endPoint
//             s1602(&params.startPoint[0], &params.endPoint[0], Order(), Dimension(), startParameter_, &endParameter_, &curve_, &statusFlag_);

//             // Pick parameters range of the line.
//             s1363(curve_, &startParameter_, &endParameter_, &statusFlag_);

//             // Pick curve length
//             s1240(curve_, Epsge(), &length_, &statusFlag_);

//             FromAbsToPos(startParameter_, startPoint_);
//             FromAbsToPos(endParameter_, endPoint_); 
//         }

//     }