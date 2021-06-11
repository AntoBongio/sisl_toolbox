#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sisl_toolbox/defines.hpp>

struct SISLCurve; /** Forward declaration */
class Curve; /** Forward declaration */
class Circle; /** Forward declaration */
class StraightLine; /** Forward declaration */
class GenericCurve; /** Forward declaration */

class CurveFactory
{
public:

    template <typename T, typename... Args>
    static std::shared_ptr<T> NewCurve(Args... args) {

        return std::make_shared<T>(args...);
        
    }


private:

};

