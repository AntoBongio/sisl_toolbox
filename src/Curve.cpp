#include "sisl_toolbox/Curve.hpp"
#include "sisl.h"

Curve::Curve(int dimension, int order, int type) 
    : dimension_{dimension}
    , order_{order}
    , type_{type}
    , epsge_{0.000001} {}


Curve::Curve(int dimension, int order, int type, SISLCurve *curve) 
    : Curve(dimension, order, type) {
        curve_ = curve;

        // Pick parameters range of the circle.
        s1363(curve_, &startParameter_, &endParameter_, &statusFlag_);

        // Pick curve length.
        s1240(curve_, Epsge(), &length_, &statusFlag_);       

        FromAbsToPos(startParameter_, startPoint_);
        FromAbsToPos(endParameter_, endPoint_);
    }


bool Curve::SaveCurve(int const samples, std::string const path, std::string const mode) const
{
    try {
        std::shared_ptr<std::ofstream> file;

        if(mode == "write")
            file = std::make_shared<std::ofstream>(path.c_str(), std::ofstream::out);
        else if(mode == "append")
            file = std::make_shared<std::ofstream>(path.c_str(), std::ofstream::app);
        if (!(*file))
            throw std::runtime_error("Unable to open Output file: " + path);

        int left{0};
        int status{0};
        double param{0};
        std::array<double, 3> pos{0};

        for (double k = 0; k < samples; ++k) {
            param = k / (samples - 1) * endParameter_;
            s1221(curve_, 0, param, &left, &pos[0], &status);
            
            *file << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
        }
        
        file->close();

    } catch (std::exception& e) {
        std::cerr << "Exception thrown: " << e.what() << std::endl;
        return -1;
    }
}

void Curve::FromAbsToPos(double abscissa, Eigen::Vector3d& worldF_position)
{
    int left{0}; // The SISL routine needs this variable, but it does not use the value.
    if(abscissa < startParameter_) abscissa = startParameter_;
    if(abscissa > endParameter_) abscissa = endParameter_;
    // The second parameters is set to zero in order to compute the position without the successive derivatives.
    s1221(curve_, 0, abscissa, &left, &worldF_position[0], &statusFlag_);
}


std::shared_ptr<Curve> Curve::ExtractCurveSection(double startValue, double endValue, double& beyondLowerLimit, double& beyondUpperLimit) {

    beyondLowerLimit = 0;
    beyondUpperLimit = 0;

    if(startValue < startParameter_)  {
        beyondLowerLimit = std::abs(startParameter_ - startValue) * (length_ / endParameter_);
        startValue = startParameter_;
    }
    if(endValue > endParameter_){
        beyondUpperLimit = std::abs(endValue - (endParameter_ - startValue)) * (length_ / endParameter_);
        endValue = endParameter_;
    } 
    SISLCurve* curveSection;
    s1712(curve_, startValue, endValue, &curveSection, &statusFlag_);

    //std::cout << "beyondUpperLimit: " << beyondUpperLimit << std::endl;

    return std::make_shared<Curve>(type_, dimension_, order_, curveSection);
}


std::tuple<double, double> Curve::FindClosestPoint(Eigen::Vector3d& worldF_position) 
{
    double distance{0};
    double abscissa{0};
    double epsco{0}; // Computational resolution (not used)

    s1957(curve_, &worldF_position[0], dimension_, epsco, epsge_, &abscissa, &distance, &statusFlag_);
    
    return std::make_tuple(abscissa, distance);
}

double Curve::AlongCurveDistance(double const abscissa) 
{
    return abscissa * (length_ / endParameter_);
}

void Curve::EvalTangentFrame(double abscissa, Eigen::Vector3d& tangent, Eigen::Vector3d& normal, Eigen::Vector3d& binormal)
{
    std::array<double, 3> worldF_position{ 0 };

    s2559(curve_, &abscissa, 1, &worldF_position[0], &tangent[0], &normal[0], &binormal[0], &statusFlag_);

    normal = tangent.cross(-Eigen::Vector3d::UnitZ());
    binormal = tangent.cross(normal);
}

void Curve::Reverse() 
{
    s1706(curve_);
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

    //std::cout << "intersectionsNum: " << intersectionsNum << ", numintcu: " << numintcu << std::endl;

    std::vector<Eigen::Vector3d> intersections{};
    Eigen::Vector3d intersectionPoint;


    for(auto i = 0; i < intersectionsNum; ++i) {
        FromAbsToPos(intersectionsFirstCurve[i], intersectionPoint);
        
        intersectionPoint[0] = std::round(intersectionPoint[0] * 1000) / 1000;
        intersectionPoint[1] = std::round(intersectionPoint[1] * 1000) / 1000;
        intersectionPoint[2] = std::round(intersectionPoint[2] * 1000) / 1000;

        if (std::count(intersections.begin(), intersections.end(), intersectionPoint) == 0) {
            intersections.push_back(intersectionPoint);
        }
    }


    return intersections;

}
