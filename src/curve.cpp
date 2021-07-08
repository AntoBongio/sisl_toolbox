#include "sisl_toolbox/curve.hpp"
#include "sisl.h"


Curve::Curve(int dimension, int order) 
    : dimension_{dimension}
    , order_{order}
    , endParameter_m_{0}
    , startParameter_m_{0}
    , length_{0}
    , epsge_{0.000001} {}


Curve::Curve(SISLCurve *curve, int dimension, int order) 
    : Curve(dimension, order) {

        curve_ = curve;

        // Pick parameters range of the curve.
        s1363(curve_, &startParameter_s_, &endParameter_s_, &statusFlag_);

        // Pick curve length.
        // s1240(curve_, Epsge(), &endParameter_m_, &statusFlag_);             
        s1240(curve_, Epsge(), &length_, &statusFlag_);  

        try {
            FromAbsSislToPos(startParameter_s_, startPoint_);
            FromAbsSislToPos(endParameter_s_, endPoint_);
        } catch(std::runtime_error const& exception) {
            throw std::runtime_error(std::string{"[Curve::Curve] -> "} + exception.what());
        }

        startParameter_m_ = startParameter_s_ * (length_ / (endParameter_s_ - startParameter_s_));
        endParameter_m_ = endParameter_s_ * (length_ / (endParameter_s_ - startParameter_s_));
    }


double Curve::SislAbsToMeterAbs(double abscissa_s) 
{
    if(endParameter_s_ >= startParameter_s_) {
        if(abscissa_s < startParameter_s_)
            throw std::runtime_error("[Curve::SislAbsToMeterAbs] Input parameter error. abscissa_s before startParameter_s_"); 
        else if (abscissa_s > endParameter_s_)
            throw std::runtime_error("[Curve::SislAbsToMeterAbs] Input parameter error. abscissa_s beyond endParameter_s_");
    }
    else {
        if(abscissa_s > startParameter_s_)
            throw std::runtime_error("[Curve::SislAbsToMeterAbs] Input parameter error. abscissa_s beyond startParameter_s_"); 
        else if (abscissa_s < endParameter_s_)
            throw std::runtime_error("[Curve::SislAbsToMeterAbs] Input parameter error. abscissa_s before endParameter_s_");
    }
 
    return abscissa_s * (length_ / (endParameter_s_ - startParameter_s_));
}


double Curve::MeterAbsToSislAbs(double abscissa_m) 
{
    if(endParameter_m_ > startParameter_m_) {
        if(abscissa_m < startParameter_m_)
            throw std::runtime_error("[Curve::MeterAbsToSislAbs] Input parameter error. abscissa_m before startParameter_m_"); 
        else if (abscissa_m > endParameter_m_)
            throw std::runtime_error("[Curve::MeterAbsToSislAbs] Input parameter error. abscissa_m beyond endParameter_m_");
    }
    else {
        if(abscissa_m > startParameter_m_)
            throw std::runtime_error("[Curve::MeterAbsToSislAbs] Input parameter error. abscissa_m beyond startParameter_m_"); 
        else if (abscissa_m < endParameter_m_)
            throw std::runtime_error("[Curve::MeterAbsToSislAbs] Input parameter error. abscissa_m before endParameter_m_");
    }
 
    return abscissa_m * ((endParameter_s_ - startParameter_s_) / length_);
}


void Curve::FromAbsSislToPos(double abscissa_s, Eigen::Vector3d& worldF_position)
{
    if(endParameter_s_ > startParameter_s_) {
        if(abscissa_s < startParameter_s_)
            throw std::runtime_error("[Curve::FromAbsSislToPos] Input parameter error. abscissa_s before startParameter_s_"); 
        else if (abscissa_s > endParameter_s_)
            throw std::runtime_error("[Curve::FromAbsSislToPos] Input parameter error. abscissa_s beyond endParameter_s_");
    }
    else {
        if(abscissa_s > startParameter_s_)
            throw std::runtime_error("[Curve::FromAbsSislToPos] Input parameter error. abscissa_s beyond startParameter_s_"); 
        else if (abscissa_s < endParameter_s_)
            throw std::runtime_error("[Curve::FromAbsSislToPos] Input parameter error. abscissa_s before endParameter_s_");
    }

    int left{0}; // The SISL routine needs this variable, but it does not use the value.
    
    s1221(curve_, 0, abscissa_s, &left, &worldF_position[0], &statusFlag_);
}


void Curve::FromAbsMetersToPos(double abscissa_m, Eigen::Vector3d& worldF_position)
{
    double abscissa_s{0};
    try {
        abscissa_s = MeterAbsToSislAbs(abscissa_m);
    }
    catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Curve::FromAbsMetersToPos] -> ") + exception.what());
    } 
    
    int left{0}; // The SISL routine needs this variable, but it does not use the value.

    s1221(curve_, 0, abscissa_s, &left, &worldF_position[0], &statusFlag_);
}


Eigen::Vector3d Curve::At(double abscissa_m) {

    Eigen::Vector3d worldF_position{};
    int leftknot{0}; // The SISL routine needs this variable, but it does not use the value.
    double abscissa_s{};
    try {
        abscissa_s = MeterAbsToSislAbs(abscissa_m);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Curve::At] -> ") + exception.what());
    }    
    
    s1227(curve_, 0, abscissa_s, &leftknot, &worldF_position[0], &statusFlag_);

    return worldF_position;
}


std::vector<Eigen::Vector3d> Curve::Derivate(int order, double abscissa_m) {

    std::vector<Eigen::Vector3d> derivates{};
    std::vector<double> derivatesTmp(order * dimension_, 0);

    int leftknot{0}; // The SISL routine needs this variable, but it does not use the value.
    double abscissa_s{};
    try {
        abscissa_s = MeterAbsToSislAbs(abscissa_m);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Curve::Derivate] -> ") + exception.what());
    }    

    s1227(curve_, order, abscissa_s, &leftknot, &derivatesTmp[0], &statusFlag_);

    for(int i = 1; i <= order; ++i) {
        derivates.emplace_back(Eigen::Vector3d{derivatesTmp[i*3], derivatesTmp[i*3 + 1], derivatesTmp[i*3 + 2]});
    }

    return derivates;
}


double Curve::Curvature(double abscissa_m) {

    double abscissa_s{0};
    try {
        abscissa_s = MeterAbsToSislAbs(abscissa_m);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Curve::Curvature] -> ") + exception.what());
    }

    std::array<double, 1> abscissaVector_s{abscissa_s};
    int parameterNumber{1};
    std::array<double, 1> curvature{};

    s2550(curve_, &abscissaVector_s[0], parameterNumber, &curvature[0], &statusFlag_);

    return curvature[0];
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


std::tuple<double, double> Curve::FindClosestPoint(Eigen::Vector3d& worldF_position) 
{

    double distance{0};
    double abscissa_s{0};
    double epsco{0}; // Computational resolution (not used)
    double abscissa_m{0};

    s1957(curve_, &worldF_position[0], dimension_, epsco, epsge_, &abscissa_s, &distance, &statusFlag_);
    try {
        abscissa_m = SislAbsToMeterAbs(abscissa_s);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Curve::FindClosestPoint] -> ") + exception.what());
    }
    
    return std::make_tuple(abscissa_m, distance);
}


std::shared_ptr<Curve> Curve::ExtractSection(double startValue_m, double endValue_m) {

    double startValue{0};
    double endValue{0};

    try {
        startValue = MeterAbsToSislAbs(startValue_m);
        endValue = MeterAbsToSislAbs(endValue_m);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string{"[Curve::ExtractSection] -> "} + exception.what());
    }
        
    SISLCurve* curveSection;
    s1712(curve_, startValue, endValue, &curveSection, &statusFlag_);

    auto curveSectionSmart = std::make_shared<Curve>(curveSection);
    curveSectionSmart->name_ = name_; 

    return curveSectionSmart;
}


std::vector<Eigen::Vector3d> Curve::Intersection(std::shared_ptr<Curve> otherCurve) {
    
    double epsco{0};
    int intersectionsNum{0};
    double * intersectionsFirstCurve; // 
    double * intersectionsSecondCurve;
    int numintcu{0};
    SISLIntcurve **intcurve;

    std::vector<Eigen::Vector3d> intersections{};
    Eigen::Vector3d intersectionPoint;

    if(otherCurve->Length() == 0)
        return intersections;

    s1857(curve_, otherCurve->CurvePtr(), epsco, epsge_, &intersectionsNum, &intersectionsFirstCurve, &intersectionsSecondCurve, 
        &numintcu, &intcurve, &statusFlag_);

    for(auto i = 0; i < intersectionsNum; ++i) {

        try {
            FromAbsSislToPos(intersectionsFirstCurve[i], intersectionPoint);
        } catch(std::runtime_error const& exception) {
            throw std::runtime_error(std::string("[Curve::Intersection] -> ") + exception.what());
        }
        
        intersectionPoint[0] = std::round(intersectionPoint[0] * 1000) / 1000;
        intersectionPoint[1] = std::round(intersectionPoint[1] * 1000) / 1000;
        intersectionPoint[2] = std::round(intersectionPoint[2] * 1000) / 1000;

        if (std::count(intersections.begin(), intersections.end(), intersectionPoint) == 0) {
            intersections.push_back(intersectionPoint);
        }
    }

    return intersections;
}













/** FIX: DA FARE!!! */
void Curve::EvalTangentFrame(double abscissa_m, Eigen::Vector3d& tangent, Eigen::Vector3d& normal, Eigen::Vector3d& binormal)
{
    
    std::array<double, 3> worldF_position{ 0 };

    double abscissa_s{};
    abscissa_s = MeterAbsToSislAbs(abscissa_m);

    s2559(curve_, &abscissa_s, 1, &worldF_position[0], &tangent[0], &normal[0], &binormal[0], &statusFlag_);

    normal = tangent.cross(-Eigen::Vector3d::UnitZ());
    binormal = tangent.cross(normal);
}