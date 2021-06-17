#include "sisl_toolbox/path.hpp"

#include "sisl_toolbox/curve.hpp" 
#include <exception>

Path::Path()
: curvesNumber_{0} 
, startParameter_m_{0}
, endParameter_m_{0}
, length_{0}
, name_ {""} {}


std::tuple<double, int> Path::PathAbsToCurveAbs(double abscissa_m) {

    if(abscissa_m < startParameter_m_){
        throw std::runtime_error("[Path::PathAbsToCurveAbs] Input parameter error. abscissa_m before startParameter_m_");
    }
    if(abscissa_m > endParameter_m_) {
        throw std::runtime_error("[Path::PathAbsToCurveAbs] Input parameter error. abscissa_m beyond endParameter_m_");
    }

    double abscissaCurve_m{};
    int curveId{0};

    double curveLength {curves_[curveId]->Length()};

    while(abscissa_m > 0) {
        if(abscissa_m > curveLength and curveId < curvesNumber_) {
            abscissa_m -= curveLength;
            ++curveId;
            curveLength = curves_[curveId]->Length();
        }
        else {
            if(curves_[curveId]->StartParameter_m() >= 0 and curves_[curveId]->EndParameter_m() >= 0) {
                if(curves_[curveId]->EndParameter_m() >= curves_[curveId]->StartParameter_m()) {
                    abscissaCurve_m = curves_[curveId]->StartParameter_m() + abscissa_m;
                }
                else {
                    abscissaCurve_m = curves_[curveId]->StartParameter_m() - abscissa_m;
                }
            }
            else if(curves_[curveId]->StartParameter_m() >= 0 and curves_[curveId]->EndParameter_m() <= 0) {
                abscissaCurve_m = curves_[curveId]->StartParameter_m() - abscissa_m;
            }
            else if(curves_[curveId]->StartParameter_m() <= 0 and curves_[curveId]->EndParameter_m() >= 0) {
                abscissaCurve_m = curves_[curveId]->StartParameter_m() + abscissa_m;
            }
            else {
                if(curves_[curveId]->EndParameter_m() <= curves_[curveId]->StartParameter_m()) {
                    abscissaCurve_m = curves_[curveId]->StartParameter_m() + abscissa_m;
                }
                else {
                    abscissaCurve_m = curves_[curveId]->StartParameter_m() - abscissa_m;
                }
            }
            abscissa_m = 0;
        }
    }
    std::cout << "Return values: (abscissaCurve_m = " << abscissaCurve_m << ", curveId = " << curveId << ")" << std::endl;
    return std::make_tuple(abscissaCurve_m, curveId);
}


double Path::CurveAbsToPathAbs(double abscissaCurve_m, int curveId) {

    double abscissa_m{0};

    if(curveId < 0 or curveId > (curvesNumber_ - 1)) 
        throw std::runtime_error(std::string("[Path::CurveAbsToPathAbs] CurveId out of bound!!"));


    // Eval abscissa_m up to the start of the curve of curveId
    for(auto i = 0; i < curveId; ++i)
        abscissa_m += curves_[i]->Length();

    if(abscissaCurve_m < curves_[curveId]->StartParameter_m()) {
        //overBound.setLower(abscissaCurve_m, curves_[curveId]->StartParameter_m());
    }
    else if (abscissaCurve_m > curves_[curveId]->EndParameter_m()) {
        //overBound.setUpper(abscissaCurve_m, curves_[curveId]->EndParameter_m());
        abscissa_m += curves_[curveId]->Length();
    }
    else {
        abscissa_m += abscissaCurve_m;
    }
    
    return abscissa_m;
}


Eigen::Vector3d Path::At(double abscissa_m) {
    Eigen::Vector3d point{};

    double abscissaCurve_m{0};
    int curveId{0};
    
    if(abscissa_m < startParameter_m_){
        throw std::runtime_error("[Path::At] Input parameter error. abscissa_m before startParameter_m_");
    }
    if(abscissa_m > endParameter_m_) {
        throw std::runtime_error("[Path::At] Input parameter error. abscissa_m beyond endParameter_m_");
    }

    std::tie(abscissaCurve_m, curveId) = PathAbsToCurveAbs(abscissa_m);

    curves_[curveId]->FromAbsMetersToPos(abscissaCurve_m, point);
    
    return point;
}

std::vector<Eigen::Vector3d> Path::Derivate(int order, double abscissa_m) {

    double abscissaCurve_m{0};
    int curveId{0};

    if(abscissa_m < startParameter_m_){
        throw std::runtime_error("[Path::Derivate] Input parameter error. abscissa_m before startParameter_m_");
    }
    if(abscissa_m > endParameter_m_) {
        throw std::runtime_error("[Path::Derivate] Input parameter error. abscissa_m beyond endParameter_m_");
    }

    std::tie(abscissaCurve_m, curveId) = PathAbsToCurveAbs(abscissa_m);
    
    return curves_[curveId]->Derivate(order, abscissaCurve_m);
}

double Path::Curvature(double abscissa_m) {

    double abscissaCurve_m{0};
    int curveId{0};

    std::tie(abscissaCurve_m, curveId) = PathAbsToCurveAbs(abscissa_m);

    return curves_[curveId]->Curvature(abscissaCurve_m);
}


std::shared_ptr<std::vector<Eigen::Vector3d>> Path::Sampling(int samples) const {

    auto path = std::make_shared<std::vector<Eigen::Vector3d>>();
    auto curve = std::make_shared<std::vector<Eigen::Vector3d>>();

    int singleCurveSamples{ static_cast<int>(samples / curvesNumber_) };

    for(int i = 0; i < curvesNumber_; ++i) {
        curve = curves_[i]->Sampling(singleCurveSamples);
        path->insert( path->end(), curve->begin(), curve->end() );
        }

    return path;
}


void Path::Reverse() {

    for(auto& elem: curves_)
        elem->Reverse();
    std::reverse(curves_.begin(), curves_.end());
}


Eigen::Vector3d Path::FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa_m) {

    Eigen::Vector3d closestPoint{Eigen::Vector3d::Zero()};
    double distance{0};
    double minDistance{0};
    double abscissaTmp_m{0};

    for(std::size_t i = 0; i < curves_.size(); ++i) {

        // try {
        //     std::tie(abscissaTmp_m, distance) = curves_[i]->FindClosestPoint(worldF_position);
        // } catch (const char* exception) {
        //     throw exception;
        // }
        
        
        try {
            std::tie(abscissaTmp_m, distance) = curves_[i]->FindClosestPoint(worldF_position);
        } catch(std::runtime_error const& exception) {
            throw std::runtime_error(std::string{"[Path::FindClosestPoint] -> "} + exception.what());
        }

        if(minDistance > 0 and distance < minDistance) {
            minDistance = distance;
            curveId = i;
            abscissa_m = abscissaTmp_m;
        }
        else if (minDistance == 0){
            minDistance = distance;
            curveId = i;
            abscissa_m = abscissaTmp_m;
        }
    }
    try {
        curves_[curveId]->FromAbsMetersToPos(abscissa_m, closestPoint);
    } catch(std::runtime_error const& exception) {
        throw std::runtime_error(std::string("[Path::FindClosestPoint] -> ") + exception.what());
    }

    return closestPoint;
}


std::shared_ptr<Path> Path::ExtractSection(double startValue_m, double endValue_m) {
    
    auto pathPortion = std::make_shared<Path>();
    double abscissaCurve_m{};
    int curveId{0};

    if(startValue_m < startParameter_m_){
        throw std::runtime_error("[Path::ExtractSection] Input parameter error. startValue_m before startParameter_m_");
        // std::cout << "Received an incorrect starting value! Fixing.." << std::endl;
        // startValue_m = startParameter_m_;
    }
    if(endValue_m > endParameter_m_) {
        throw std::runtime_error("[Path::ExtractSection] Input parameter error. endValue_m beyond endParameter_m_");
        // std::cout << "Received an incorrect ending value! Fixing.." << std::endl;
        // endValue_m = endParameter_m_;
    }

    double portionLength{endValue_m - startValue_m};

    std::tie(abscissaCurve_m, curveId) = PathAbsToCurveAbs(startValue_m);

    if(portionLength < curves_[curveId]->EndParameter_m() - abscissaCurve_m) {

        try {
            pathPortion->AddCurveBack<Curve>(curves_[curveId]->ExtractSection(
            abscissaCurve_m, portionLength));
        } catch (std::runtime_error const& exception) {
            throw std::runtime_error(std::string("[Path::ExtractSection] -> ") + exception.what());
        }       

        portionLength = 0;
    }
    else {
        try {
            pathPortion->AddCurveBack<Curve>(curves_[curveId]->ExtractSection(
                abscissaCurve_m, curves_[curveId]->EndParameter_m()));
        } catch (std::runtime_error const& exception) {
            throw std::runtime_error(std::string("[Path::ExtractSection] -> ") + exception.what());
        }

        portionLength -= curves_[curveId]->EndParameter_m() - abscissaCurve_m;
        ++curveId;
    }

    while(portionLength > 0 and curveId < curvesNumber_ ) {
    
        if(portionLength < curves_[curveId]->EndParameter_m()) {
            
            try {
                pathPortion->AddCurveBack<Curve>(curves_[curveId]->ExtractSection(curves_[curveId]->StartParameter_m(), 
                    portionLength));
            } catch (std::runtime_error const& exception) {
                throw std::runtime_error(std::string("[Path::ExtractSection] -> ") + exception.what());
            }

            portionLength = 0;
        }
        else {
            pathPortion->AddCurveBack<Curve>(curves_[curveId]);
            portionLength -= curves_[curveId]->EndParameter_m();
            ++curveId;
        }
    }
    
    return pathPortion;
}



std::vector<Eigen::Vector3d> Path::Intersection(std::shared_ptr<Path> otherPath) {

    std::vector<Eigen::Vector3d> intersections;

    for(auto const & curve: curves_) {
        for(auto const & otherCurve: otherPath->Curves()) {
            
            std::vector<Eigen::Vector3d> intersectionPoints;
            try {
                intersectionPoints = curve->Intersection(otherCurve);
            } catch (std::runtime_error const& exception) {
                throw std::runtime_error(std::string("[Path::Intersection] -> ") + exception.what());
            }
            
            for (auto const & point: intersectionPoints) {
                if (std::count(intersections.begin(), intersections.end(), point) == 0) {
                    intersections.push_back(point);
                }
            }
        }
    }

    return intersections;
}


std::vector<Eigen::Vector3d> Path::Intersection(int curveId, std::shared_ptr<Path> otherPath) {

    std::vector<Eigen::Vector3d> intersections;

    if(curveId > curvesNumber_ - 1)
        return intersections;
    
    std::vector<Eigen::Vector3d> intersectionPoints;

    for(auto const & otherCurve: otherPath->Curves()) {

        try {

            intersectionPoints = curves_[curveId]->Intersection(otherCurve);

        } catch (std::runtime_error const& exception) {
            throw std::runtime_error(std::string("[Path::Intersection] -> ") + exception.what());
        }

        for (auto const & point: intersectionPoints) {
            if (std::count(intersections.begin(), intersections.end(), point) == 0) {
                intersections.push_back(point);
            }
        }
    }
    
    return intersections;
}


std::vector<Eigen::Vector3d> Path::Intersection(std::shared_ptr<Curve> otherCurve) {

    std::vector<Eigen::Vector3d> intersections;
    std::vector<Eigen::Vector3d> intersectionPoints;

    for(auto const & curve: curves_) {

        try {

            intersectionPoints = curve->Intersection(otherCurve);

        } catch (std::runtime_error const& exception) {
            throw std::runtime_error(std::string("[Path::Intersection] -> ") + exception.what());
        }

        for (auto const & point: intersectionPoints) {
            if (std::count(intersections.begin(), intersections.end(), point) == 0) {
                intersections.push_back(point);
            }
        }
    }
    
    return intersections;
}






std::tuple<double, double, double, double> Path::evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces) const {
        double maxX{polygonVerteces[0][0]}; 
        double minX{polygonVerteces[0][0]};
        double maxY{polygonVerteces[0][1]};
        double minY{polygonVerteces[0][1]};
        for(std::size_t i = 1; i < polygonVerteces.size(); i++) {
            if(maxX < polygonVerteces[i][0])
                maxX = polygonVerteces[i][0];
            if(minX > polygonVerteces[i][0])
                minX = polygonVerteces[i][0];   

            if(maxY < polygonVerteces[i][1])
                maxY = polygonVerteces[i][1];
            if(minY > polygonVerteces[i][1])
                minY = polygonVerteces[i][1];  
        }
        // Set the vertices precision
        return std::make_tuple(std::round(maxX * 1000) / 1000, std::round(minX * 1000) / 1000, 
                               std::round(maxY * 1000) / 1000, std::round(minY * 1000) / 1000);
    };