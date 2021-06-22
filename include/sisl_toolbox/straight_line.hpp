#pragma once

#include "sisl_toolbox/curve.hpp"

/**
 * @class StraightLine
 *
 * @brief Class derived from Curve. It adds a specific constructor for Straight Line Curve objects as well as the specific getters. 
 */
class StraightLine : public Curve{

public:

    /** 
     * @brief Straight Line constructor based on s1602() SISL routine.
     * 
     * @param startPoint Start point of the straight line.
     * @param endPoint End point of the straight line.
     * 
     * @param dimension Parameter used in Curve constructor -> default = 3
     * @param order Parameter used in Curve constructor -> default = 3
     */
    StraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, int dimension = 3, int order = 3);
    

private:    
    
};
