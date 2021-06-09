#ifndef STRAIGHTLINE_HPP
#define STRAIGHTLINE_HPP

#include "sisl_toolbox/curve.hpp"
#include "sisl_toolbox/defines.hpp"

class StraightLine : public Curve{

public:

    /** 
     * @brief StraightLine constructor based on s1602() SISL routine.
     * 
     * @param startPoint Start point of the straight line.
     * @param endPoint End point of the straight line.
     * 
     * @param dimension Parameter used in Curve constructor -> default = 3
     * @param order Parameter used in Curve constructor -> default = 3
     */
    StraightLine(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, int dimension = 3, int order = 3);
    /** PROBLEMA: Non riesco a metterla privata perchè CurveFactory non la vede!! */
    

private:

    friend class CurveFactory;
    
    
};
#endif
