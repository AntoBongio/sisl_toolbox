#include "sisl_toolbox/curve.hpp"
#include "sisl.h"


Curve::Curve(int type, SISLCurve *curve, int dimension, int order) 
    : Curve(type, dimension, order) {

        curve_ = curve;

        // Pick parameters range of the curve.
        s1363(curve_, &startParameter_s_, &endParameter_s_, &statusFlag_);

        // Pick curve length.
        s1240(curve_, Epsge(), &endParameter_m_, &statusFlag_);       

        FromAbsSislToPos(startParameter_s_, startPoint_);
        FromAbsSislToPos(endParameter_s_, endPoint_);

        startParameter_m_ = startParameter_s_ * (endParameter_m_ / endParameter_s_);

    }

Curve::Curve(int dimension, int order, int type) 
    : dimension_{dimension}
    , order_{order}
    , type_{type}
    , endParameter_m_{0}
    , startParameter_m_{0}
    , epsge_{0.000001} {}



// void Curve::FromAbsToPos(double abscissa, Eigen::Vector3d& worldF_position)
// {
//     int left{0}; // The SISL routine needs this variable, but it does not use the value.
//     if(abscissa < startParameter_s_) abscissa = startParameter_s_;
//     if(abscissa > endParameter_s_) abscissa = endParameter_s_;
//     // The second parameters is set to zero in order to compute the position without the successive derivatives.

    
//     s1221(curve_, 0, abscissa, &left, &worldF_position[0], &statusFlag_);
// }







////////////////////////////////////////////////

double Curve::AlongCurveDistance(double const abscissa) 
{
    return abscissa * (endParameter_m_ / endParameter_s_);
}

////////////////////////////////////////////////



std::shared_ptr<Curve> Curve::ExtractCurveSection(double startValue_m, double endValue_m, double& beyondLowerLimit, double& beyondUpperLimit) {

    double startValue{0};
    std::tie(startValue, std::ignore) = MeterAbsToSislAbs(startValue_m);
    double endValue{0};
    std::tie(endValue, std::ignore) = MeterAbsToSislAbs(endValue_m);

    beyondLowerLimit = 0;
    beyondUpperLimit = 0;

    if(startValue < startParameter_s_)  {
        beyondLowerLimit = std::abs(startParameter_s_ - startValue) * (endParameter_m_ / endParameter_s_);
        startValue = startParameter_s_;
    }
    if(endValue > endParameter_s_){
        beyondUpperLimit = std::abs(endValue - (endParameter_s_ - startValue)) * (endParameter_m_ / endParameter_s_);
        endValue = endParameter_s_;
    } 
    SISLCurve* curveSection;
    s1712(curve_, startValue, endValue, &curveSection, &statusFlag_);

    auto curvePtr = Curve(type_, curveSection);
    return std::make_shared<Curve>(curvePtr);
}


std::tuple<double, double> Curve::FindClosestPoint(Eigen::Vector3d& worldF_position) 
{
    double distance{0};
    double abscissa_s{0};
    double epsco{0}; // Computational resolution (not used)
    double abscissa_m{0};

    s1957(curve_, &worldF_position[0], dimension_, epsco, epsge_, &abscissa_s, &distance, &statusFlag_);
    std::tie(abscissa_m, std::ignore) = SislAbsToMeterAbs(abscissa_s);
    
    return std::make_tuple(abscissa_m, distance);
}


void Curve::EvalTangentFrame(double abscissa_m, Eigen::Vector3d& tangent, Eigen::Vector3d& normal, Eigen::Vector3d& binormal)
{
    std::array<double, 3> worldF_position{ 0 };

    double abscissa_s{};
    std::tie(abscissa_s, std::ignore) = MeterAbsToSislAbs(abscissa_m);

    s2559(curve_, &abscissa_s, 1, &worldF_position[0], &tangent[0], &normal[0], &binormal[0], &statusFlag_);

    normal = tangent.cross(-Eigen::Vector3d::UnitZ());
    binormal = tangent.cross(normal);
}


std::vector<Eigen::Vector3d> Curve::Intersection(std::shared_ptr<Curve> otherCurve) {
    
    double epsco{0};
    int intersectionsNum{0};
    double * intersectionsFirstCurve; // 
    double * intersectionsSecondCurve;
    int numintcu{0};
    SISLIntcurve **intcurve;

    s1857(curve_, otherCurve->CurvePtr(), epsco, epsge_, &intersectionsNum, &intersectionsFirstCurve, &intersectionsSecondCurve, 
        &numintcu, &intcurve, &statusFlag_);

    std::vector<Eigen::Vector3d> intersections{};
    Eigen::Vector3d intersectionPoint;

    for(auto i = 0; i < intersectionsNum; ++i) {
        FromAbsSislToPos(intersectionsFirstCurve[i], intersectionPoint);
        
        intersectionPoint[0] = std::round(intersectionPoint[0] * 1000) / 1000;
        intersectionPoint[1] = std::round(intersectionPoint[1] * 1000) / 1000;
        intersectionPoint[2] = std::round(intersectionPoint[2] * 1000) / 1000;

        if (std::count(intersections.begin(), intersections.end(), intersectionPoint) == 0) {
            intersections.push_back(intersectionPoint);
        }
    }

    return intersections;
}


void Curve::FromAbsSislToPos(double abscissa_s, Eigen::Vector3d& worldF_position)
{
    int left{0}; // The SISL routine needs this variable, but it does not use the value.
    if(abscissa_s < startParameter_s_) abscissa_s = startParameter_s_;
    if(abscissa_s > endParameter_s_) abscissa_s = endParameter_s_;
    
    s1221(curve_, 0, abscissa_s, &left, &worldF_position[0], &statusFlag_);
}

void Curve::FromAbsMetersToPos(double abscissa_m, Eigen::Vector3d& worldF_position)
{
    int left{0}; // The SISL routine needs this variable, but it does not use the value.
    double abscissa_s{};
    std::tie(abscissa_s, std::ignore) = MeterAbsToSislAbs(abscissa_m);
    
    s1221(curve_, 0, abscissa_s, &left, &worldF_position[0], &statusFlag_);
}

Eigen::Vector3d Curve::At(double abscissa_m) {

    Eigen::Vector3d worldF_position{};
    int left{0}; // The SISL routine needs this variable, but it does not use the value.
    double abscissa_s{};
    std::tie(abscissa_s, std::ignore) = MeterAbsToSislAbs(abscissa_m);
    
    s1221(curve_, 0, abscissa_s, &left, &worldF_position[0], &statusFlag_);

    return worldF_position;
}


std::tuple<Eigen::Vector3d, double, overBound> Curve::MovePoint(double startValueAbscissa_m, double offset_m) {

    overBound overBound{};
    double startValueAbscissa_s{};
    double abscissa_s{};
    Eigen::Vector3d point{};
    double abscissa_m{};

    // Starting from the value received in meters, obtain the value in Sisl parametrization
    std::tie(startValueAbscissa_s, overBound) = MeterAbsToSislAbs(startValueAbscissa_m);

    // Displace keeping the curve parametrization
    abscissa_s = startValueAbscissa_s + offset_m * (endParameter_s_ / endParameter_m_);
    std::tie(abscissa_m, overBound) = SislAbsToMeterAbs(abscissa_s);

    // Obtain the point starting from the 
    
    //FromAbsMetersToPos(abscissa_m, point); // sostituito con At

    return std::make_tuple(At(abscissa_m), abscissa_m, overBound);
}


std::tuple<double, overBound> Curve::SislAbsToMeterAbs(double abscissa_s) 
{
    double abscissa_m { abscissa_s * (endParameter_m_ / endParameter_s_) };
    overBound overBound;

    if(abscissa_s < startParameter_s_) {
        overBound.setLower(abscissa_m, startParameter_m_);
        abscissa_m = startParameter_m_;
    }
    else if(abscissa_s > endParameter_s_) {
        overBound.setUpper(abscissa_m, endParameter_m_);
        abscissa_m = endParameter_m_;
    }   
 
    return std::make_tuple(abscissa_m, overBound);
}


std::tuple<double, overBound> Curve::MeterAbsToSislAbs(double abscissa_m) 
{
    double abscissa_s { abscissa_m * (endParameter_s_ / endParameter_m_) };
    overBound overBound;

    if(abscissa_m < startParameter_m_) {
        overBound.setLower(abscissa_m, startParameter_m_);
        abscissa_s = startParameter_s_;
    }
    else if(abscissa_s > endParameter_s_) {
        overBound.setUpper(abscissa_m, endParameter_m_);
        abscissa_s = endParameter_s_;
    }   
 
    return std::make_tuple(abscissa_s, overBound);
}


void Curve::Reverse() 
{
    s1706(curve_);
}


std::shared_ptr<std::vector<Eigen::Vector3d>> Curve::Sampling(int const samples) const
{
    auto curve = std::make_shared<std::vector<Eigen::Vector3d>>();

    int left{0};
    int status{0};
    double param{0};
    std::array<double, 3> pos{0};

    for (double k = 0; k < samples; ++k) {
        param = startParameter_s_ + k / (samples - 1) * (endParameter_s_ - startParameter_s_);
        s1221(curve_, 0, param, &left, &pos[0], &status);
        
        curve->emplace_back(Eigen::Vector3d{pos[0], pos[1], pos[2]});
    }
    
    return curve;
}