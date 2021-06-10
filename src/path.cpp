#include "sisl_toolbox/path.hpp"


Path::Path()
: curvesNumber_{0} 
, startParameter_m_{0}
, endParameter_m_{0}
, name_ {""} {}

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

    int count = 1;
    for(auto i = 0; i < curves_.size(); ++i) {
        std::tie(abscissaTmp_m, distance) = curves_[i]->FindClosestPoint(worldF_position);

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
    curves_[curveId]->FromAbsMetersToPos(abscissa_m, closestPoint);

    return closestPoint;
}


std::tuple<double, int, overBound> Path::PathAbsToCurveAbs(double abscissa_m) {

    overBound overBound{};
    double abscissaCurve_m{};
    int curveId{0};

    if(abscissa_m < startParameter_m_) {
        // std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m < startParameter_m_" << std::endl;
        overBound.setLower(abscissa_m, startParameter_m_);
        curveId = 0;
        abscissaCurve_m = curves_[curveId]->StartParameter_m();
    }
    else if(abscissa_m > endParameter_m_) {
        // std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m > endParameter_m_" << std::endl;
        overBound.setUpper(abscissa_m, endParameter_m_);
        curveId = curvesNumber_ - 1;
        abscissaCurve_m = curves_[curveId]->EndParameter_m();
    }
    else {
        // std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in else" << std::endl;
        while(abscissa_m > 0) {
            if(abscissa_m > curves_[curveId]->EndParameter_m()) {
                abscissa_m -= curves_[curveId]->EndParameter_m();
                ++curveId;
            }
            else {
                abscissaCurve_m = abscissa_m;
                abscissa_m = 0;
            }
        }
    }
    return std::make_tuple(abscissaCurve_m, curveId, overBound);
}


std::tuple<double, overBound> Path::CurveAbsToPathAbs(double abscissaCurve_m, int curveId) {

    overBound overBound{};
    double abscissa_m{0};
    if(curveId < 0 or curveId > (curvesNumber_ - 1)) {
        std::cout << "CurveId error!!!" << std::endl;
        return std::make_tuple(abscissa_m, overBound);
    }

    // Eval abscissa_m up to the start of the curve of curveId
    for(auto i = 0; i < curveId; ++i)
        abscissa_m += curves_[i]->Length();

    if(abscissaCurve_m < curves_[curveId]->StartParameter_m()) {
        overBound.setLower(abscissaCurve_m, curves_[curveId]->StartParameter_m());

    }
    else if (abscissaCurve_m > curves_[curveId]->EndParameter_m()) {
        overBound.setUpper(abscissaCurve_m, curves_[curveId]->EndParameter_m());
        abscissa_m += curves_[curveId]->Length();
    }
    else {
        abscissa_m += abscissaCurve_m;
    }
    
    return std::make_tuple(abscissa_m, overBound);
}


Eigen::Vector3d Path::At(double abscissa_m) {
    Eigen::Vector3d point{};

    overBound overBound{};
    double abscissaCurve_m{0};
    int curveId{0};

    std::tie(abscissaCurve_m, curveId, overBound) = PathAbsToCurveAbs(abscissa_m);

    curves_[curveId]->FromAbsMetersToPos(abscissaCurve_m, point);
    
    return point;
}


std::shared_ptr<Path> Path::ExtractSection(double startValue_m, double endValue_m) {
    
    auto pathPortion = std::make_shared<Path>();
    double abscissaCurve_m{};
    int curveId{0};

    if(startValue_m < startParameter_m_){
        std::cout << "Received an incorrect starting value! Fixing.." << std::endl;
        startValue_m = startParameter_m_;
    }
    if(endValue_m > endParameter_m_) {
        std::cout << "Received an incorrect ending value! Fixing.." << std::endl;
        endValue_m = endParameter_m_;
    }

    double portionLength{endValue_m - startValue_m};

    double beyondLowerLimit{};
    double beyondUpperLimit{};

    std::tie(abscissaCurve_m, curveId, std::ignore) = PathAbsToCurveAbs(startValue_m);

    if(portionLength < curves_[curveId]->EndParameter_m() - abscissaCurve_m) {
        pathPortion->AddCurveBack<Curve>(curves_[curveId]->ExtractCurveSection(
            abscissaCurve_m, portionLength, beyondLowerLimit, beyondUpperLimit));
        portionLength = 0;
    }
    else {
        pathPortion->AddCurveBack<Curve>(curves_[curveId]->ExtractCurveSection(
        abscissaCurve_m, curves_[curveId]->EndParameter_m(), beyondLowerLimit, beyondUpperLimit));
        portionLength -= curves_[curveId]->EndParameter_m() - abscissaCurve_m;
        ++curveId;
    }

    while(portionLength > 0 and curveId < curvesNumber_ ) {
    
        if(portionLength < curves_[curveId]->EndParameter_m()) {
            pathPortion->AddCurveBack<Curve>(
                curves_[curveId]->ExtractCurveSection(curves_[curveId]->StartParameter_m(), portionLength, beyondLowerLimit, beyondUpperLimit));
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

            auto intersectionPoints = curve->Intersection(otherCurve);

            for (auto const & point: intersectionPoints) {

                for(auto const & intersection: intersections) {
                }
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

    for(auto const & otherCurve: otherPath->Curves()) {

        auto intersectionPoints = curves_[curveId]->Intersection(otherCurve);

        for (auto const & point: intersectionPoints) {

            for(auto const & intersection: intersections) {
            }
            if (std::count(intersections.begin(), intersections.end(), point) == 0) {
                intersections.push_back(point);
            }
        }
    }
    
    return intersections;
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


std::tuple<double, double, double, double> Path::evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces) const {
        double maxX{polygonVerteces[0][0]}; 
        double minX{polygonVerteces[0][0]};
        double maxY{polygonVerteces[0][1]};
        double minY{polygonVerteces[0][1]};
        for(auto i = 1; i < polygonVerteces.size(); i++) {
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