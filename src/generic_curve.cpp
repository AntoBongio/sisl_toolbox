#include "sisl_toolbox/generic_curve.hpp"
#include "sisl.h"

GenericCurve::GenericCurve(int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights, 
    std::vector<double> coefficients, int dimension, int order)
    : Curve(dimension, order)
    , degree_{degree}
    , knots_{knots}
    , points_{points}
    , weights_{weights}
    , coefficients_{coefficients}
    {
        name_ = "Generic Curve";

        int kind{2}; /* Type of curve.
                    = 1 : Polynomial B-spline curve.
                    = 2 : Rational B-spline (nurbs) curve.
                    = 3 : Polynomial Bezier curve.
                    = 4 : Rational Bezier curve*/

        int copy{1}; /* Flag
                    = 0 : Set pointer to input arrays.
                    = 1 : Copy input arrays.
                    = 2 : Set pointer and remember to free arrays. */


        if(coefficients_.empty()) {
            for(std::size_t i = 0; i < points_.size(); ++i) {
                coefficients_.push_back(points_[i][0] * weights_[i]);
                coefficients_.push_back(points_[i][1] * weights_[i]);
                coefficients_.push_back(0);
                coefficients_.push_back(weights_[i]);
            }
        }

        curve_ = newCurve(points_.size(), degree_ + 1, &knots_[0], &coefficients_[0], kind, Dimension(), copy);

        // Pick parameters range of the circle.
        s1363(curve_, &startParameter_s_, &endParameter_s_, &statusFlag_);

        // Pick curve length.
        s1240(curve_, Epsge(), &endParameter_m_, &statusFlag_);  

        FromAbsSislToPos(startParameter_s_, startPoint_);
        FromAbsSislToPos(endParameter_s_, endPoint_);    

        startParameter_m_ = startParameter_s_ * (endParameter_m_ / endParameter_s_); 

        length_ = std::abs(endParameter_m_ - startParameter_m_);
    }