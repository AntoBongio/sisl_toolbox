#ifndef CURVE_HPP
#define CURVE_HPP

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Dense>

struct SISLCurve; /** Forward declaration */
class CurveFactory; /** Forward declaration */

/**
 * @class Curve (virtual class)
 *
 * @brief Interface for a curve. 
 * @details Practically speaking, the main objective of this interface is to define a wrapper for the most used SISL functions,
 * in order to enhance teir readability and avoiding the need of the manual. 
 */
class Curve {

public:
    /** 
     * @brief Curve constructor.
     * 
     * @param dimension Define dimension of curve
     * @param order Define order of curve
     * @param type Curve type -> 0 : Generic Curve ; 1 : Straight Line ; 2 : Circle
     */ 
    Curve(int type, int dimension = 3, int order = 3);

    /** 
     * @brief Curve constructor.
     * 
     * @param type Curve type -> 0 : Generic Curve ; 1 : Straight Line ; 2 : Circle
     * @param curve Pointer to the curve 
     * @param  dimension Define dimension of curve
     * @param order Define order of curve
     * 
     */ 
    Curve(int type, SISLCurve * curve, int dimension = 3, int order = 3);

    /**
    * @brief Save the curve in a file provided by the path.
    * @param[in] samples Number of points to describe the curve.
    * @param[in] path Path pointing to the saving location. Remember to add the file name at the end.
    * e.g.: "/home/antonino/Desktop/curve.txt".
    * @param[in] mode To select "write" or "append" mode.
    */
    bool SaveCurve(int const samples, std::string const path, std::string const mode) const; // DA ELIMINARE!!!!

    /**
    * @brief Compute the position and the right-hand derivatives of a curve at a given parameter value.
    * @details To compute the positione and the first derivatives of a curve at a given parameter value. Evaluation from the right hand side.
    * @param[in] abscissa The parameter value where to compute the position.
    * @param[out] worldF_position The point corresponding at the abscissa expressed in world frame.
    */
    void FromAbsToPos(double abscissa, Eigen::Vector3d& worldF_position);

    /**
    * @brief Pick a part of a curve.
    * @details It extracts a new curve from the stating one according to the abscissa startValue and endValue.
    * @param[in] startValue Start parameter value of the part curve to be picked.
    * @param[in] endValue End parameter value of the part curve to be picked.
    * @param[out] beyondLowerLimit Distance in meters beyond the upper limit of the curve
    * @param[out] beyondUpperLimit Distance in meters beyond the lower limit of the curve
    * 
    * @return A shared ptr to the curve object.
    */
    std::shared_ptr<Curve> ExtractCurveSection(double startValue, double endValue, double& beyondLowerLimit, double& beyondUpperLimit);
    
    /**
    * @brief Find the closest point between a curve and a point. Simple version.
    * @details Find the closest point between a curve and a point. The method is fast and should work well in clear cut cases but does not guarantee 
    * finding the right solution. As long as it doesn’t fail, it will find exactly one point. In other cases, use s1953().
    * @param[in] position The point in the closest point problem.
    * 
    * @return A tuple (double, double) containing as first element the abscissa of the on curve point solution of the closest point problem. 
    * The second element is the distance between the point passed as argument (worldF_position) and the point on curve solution of the closest point problem.
    */
    std::tuple<double, double> FindClosestPoint(Eigen::Vector3d& worldF_position);

    /**
    * @brief Transform the abscissa value into a distance in meters from the starting point.
    * @param[in] abscissa Parameter value up to which you want to calculate the distance in meters.
    * 
    * @return The along curve distance in meters from the stating point of the curve to abscissa parameter.
    */
    double AlongCurveDistance(double const abscissa);

    /**
    * @brief Transform the abscissa value into a distance in meters from the starting point.
    * @details The tangntial direction depends on the direction of the curve. Starting from this vector, the normal component is calculated as cross product among 
    * the tangent vector and the z versor of the world frame. The binormal direction is compute as cross product among tangent and normal direction.
    * @param[in] abscissa Parameter value where to calculate the tangent frame.
    * @param[out] tangent Tangent component of the tangent 3D frame.
    * @param[out] normal Normal component of the tangent 3D frame.
    * @param[out] binormal Binormal component of the tangent 3D frame.
    */
    void EvalTangentFrame(double abscissa, Eigen::Vector3d& tangent, Eigen::Vector3d& normal, Eigen::Vector3d& binormal);

    /**
    * @brief Turns the direction of the orginal curve.
    */
    void Reverse();

    std::vector<Eigen::Vector3d> Intersection(std::shared_ptr<Curve> otherCurve);

    std::shared_ptr<std::vector<Eigen::Vector3d>> Sampling(int const samples) const;


    // Getter / Setter
    auto Dimension() const& {return dimension_;}

    auto Order() const& {return order_;}

    auto Epsge() const& {return epsge_;}

    auto Type() const& {return type_;}

    auto CurvePtr() const& {return curve_;}
    
    auto StatusFlag() const& {return statusFlag_;}

    auto StartParameter() const& {return startParameter_;}

    auto EndParameter() const& {return endParameter_;}

    auto Length() const& {return length_;}

    auto StartPoint() const& {return startPoint_;}
    auto StartPoint() & {return startPoint_;}
    auto StartPoint() && {return std::move(startPoint_);}

    auto EndPoint() const& {return endPoint_;}
    auto EndPoint() & {return endPoint_;}
    auto EndPoint() && {return std::move(endPoint_);}

    

private:

    // Setter
    auto Dimension(int dimension) { dimension_ = dimension;}
    auto Order(int order) {order_ = order;}
    auto Epsge(double epsge) { epsge_ = epsge;}
    auto Type(int type) & {type_ = type;}
    auto CurvePtr(SISLCurve* curve) & {curve_ = curve;}
    auto StatusFlag(bool statusFlag) & {statusFlag_ = statusFlag;}
    auto StartParameter(double startParamenter) & { startParameter_ = startParamenter;}
    auto EndParameter(double endParameter) & { endParameter_ = endParameter;}
    auto Length(double length) & { length_ = length;}

    friend class CurveFactory;

    int dimension_; // Dimension of the curve
    int order_; // Order of the curve
    double epsge_; // Geometric resolution
    int type_;

protected:
    SISLCurve *curve_;
    int statusFlag_; // Control flag used as output of each SISL function

    double startParameter_; // Start value of the curve parametrization
    double endParameter_; // Last value of the curve parametrization
    double length_; // Length of the curve
    Eigen::Vector3d startPoint_; // Curve start point
    Eigen::Vector3d endPoint_; // Curve end point

};

/*** NOTE: mkdir build -> cmake .. -DBUILD_TESTS=ON -> sudo make install ***/

#endif