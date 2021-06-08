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

        // switch(type) {
        //     case 0:
        //         std::cout << "[CurveFactory] -> Building a GenericCurve" << std::endl;
        //         return std::make_shared<T>(args...);
        //         break;
        //     case 1:
        //     {
        //         std::cout << "[CurveFactory] -> Building a StraightLine" << std::endl;
        //         return std::make_shared<T>(args...);
        //         break;
        //     } 
        //     case 2:
        //         std::cout << "[CurveFactory] -> Building a Circle" << std::endl; 
        //         return std::make_shared<T>(args...);
        //         break;
        // }
    }


private:

};

