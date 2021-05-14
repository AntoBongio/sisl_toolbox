#ifndef STRAIGHTLINE_HPP
#define STRAIGHTLINE_HPP

#include "sisl_toolbox/Curve.hpp"

class StraightLine : public Curve{

public:
    /** 
     * @brief StraightLine constructor based on s1602() SISL routine.
     * 
     * @param type Curve type -> 0 : Generic Curve ; 1 : Straight Line ; 2 : Circle
     * @param dimension Parameter used in Curve constructor
     * @param order Parameter used in Curve constructor
     * 
     * @param startPoint Start point of the straight line.
     * @param endPoint End point of the straight line.
     */ 
    StraightLine(int type, int dimension, int order, Eigen::Vector3d startPoint, Eigen::Vector3d endPoint);

    // Getter / Setter methods

private:
};
#endif
