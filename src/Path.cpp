#include "sisl_toolbox/Path.hpp"

#include <iomanip>

Path::Path()
: curvesNumber_{0} 
, length_{0} 
, currentAbscissa_ {0}
, currentCurveId_{0} {}

Path::Path(std::vector<Parameters> & parameters)
    : curvesNumber_{0} 
    , length_{0}
    , currentAbscissa_ {0}
    , currentCurveId_{0} {

    for(const auto& elem: parameters) {
        switch(elem.type){
            case 0 : // Generic Curve
            {   
                ++curvesNumber_;
                curves_.push_back(std::make_shared<GenericCurve>(elem.type, elem.dimension, elem.order, elem.degree, elem.knots, 
                    elem.points, elem.weights, elem.coefficients));
                break;
            }
            case 1 : // StraightLine
            {   
                ++curvesNumber_;
                curves_.push_back(std::make_shared<StraightLine>(elem.type, elem.dimension, elem.order, elem.startPoint, elem.endPoint));
                break;
            }
            case 2 : // Circle
            {
                ++curvesNumber_;
                curves_.push_back(std::make_shared<Circle>(elem.type, elem.dimension, elem.order, elem.angle, elem.axis, elem.startPoint, elem.centrePoint));
                break;
            }
        }
    }

    for (const auto & curve: curves_) {
        length_ += curve->Length();
    }

    currentAbscissa_ = curves_[0]->StartParameter();
}

Path::Path(std::vector<Eigen::Vector3d>& points)
    : curvesNumber_{0} 
    , length_{0} {

    for(int i = 0; i < points.size() - 1; ++i) {
        std::cout << "Segment(" << i << ") -> From [" << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << "] to [" 
            << points[i+1][0] << ", " << points[i+1][1] << ", " << points[i+1][2] << "]" << std::endl;
        curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, points[i], points[i+1]));
        ++curvesNumber_;
    }
    std::cout << "Segment(" << points.size() - 1 << ") -> From [" << points[points.size() - 1][0] << ", " 
        << points[points.size() - 1][1] << ", " << points[points.size() - 1][2] << "] to [" 
        << points[0][0] << ", " << points[0][1] << ", " << points[0][2] << "]" << std::endl;
    curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, points[points.size() - 1], points[0]));
    ++curvesNumber_;
    /*
    for(auto it = points.begin(); it != points.end(); ++it) {
        std::cout << "First: " << (*it)[0] << ", second: " << (*(++it))[0] << std::endl; 
        curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, *it, *(++it)));
        ++curvesNumber_;
    }
    */

    for (const auto & curve: curves_) {
        length_ += curve->Length();
    }

    std::cout << "curvesNumber_: " << curvesNumber_ << std::endl;

}

Path::Path(double angle, double offset, std::vector<Eigen::Vector3d>& polygonVerteces)
    : curvesNumber_{0} 
    , length_{0} {

    auto polygon = std::make_shared<Path>(polygonVerteces);
    polygon->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/polygon.txt");

    // Compute the rectangle surrounding the polygon
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
    std::vector<Eigen::Vector3d> rectangleVertices{Eigen::Vector3d{maxX, maxY, 0}, Eigen::Vector3d{maxX, minY, 0},
                                                   Eigen::Vector3d{minX, minY, 0}, Eigen::Vector3d{minX, maxY, 0}};
    auto rectangle = std::make_shared<Path>(rectangleVertices);
    rectangle->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/rectangle.txt");


    auto parallelStraightLines = std::make_shared<Path>();

    /** TODO: COntinuare da qua */
    double movingX{maxX};
    double movingY{minY};
    while(movingX > minX and movingY < maxY) {
        movingX -= std::cos(-angle) * offset;
        movingY += std::sin(-angle) * offset;
        std::cout << "minX: "<< minX << ", movingX: " << movingX << ", maxY: "<< maxY << ", movingY: " << movingY << std::endl;
        auto line = std::make_shared<StraightLine>(1, 3, 3, Eigen::Vector3d{movingX, minY, 0}, Eigen::Vector3d{maxX, movingY, 0});
        parallelStraightLines->AddCurveBack(line);
    }
    

    parallelStraightLines->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/parallelStraightLines.txt");

}


template <typename T>
void Path::AddCurveBack(std::shared_ptr<T> curve) {
    curves_.push_back(curve);
    std::cout << "Curve length: " << curve->Length() << std::endl;
    length_ += curve->Length();
    ++curvesNumber_; 
}


void Path::SavePath(int samples, std::string const path) const {

    int singleCurveSamples{ static_cast<int>(samples / curvesNumber_) };

    for(int i = 0; i < curvesNumber_; ++i) {
        if (i == 0)
            curves_[i]->SaveCurve(singleCurveSamples, path, "write");
        else
            curves_[i]->SaveCurve(singleCurveSamples, path, "append");
    }
}

void Path::Reverse() {

    for(auto& elem: curves_)
        elem->Reverse();
    std::reverse(curves_.begin(), curves_.end());
}

Eigen::Vector3d Path::FindClosestPoint(Eigen::Vector3d& worldF_position, int& curveId, double& abscissa) {

    Eigen::Vector3d closestPoint{Eigen::Vector3d::Zero()};
    double distance{0};
    double minDistance{0};
    double abscissaTmp{0};

    int count = 1;
    for(auto i = 0; i < curves_.size(); ++i) {
        std::tie(abscissaTmp, distance) = curves_[i]->FindClosestPoint(worldF_position);

        if(minDistance > 0 and distance < minDistance) {
            minDistance = distance;
            curveId = i;
            abscissa = abscissaTmp;
        }
        else if (minDistance == 0){
            minDistance = distance;
            curveId = i;
            abscissa = abscissaTmp;
        }
    }
    curves_[curveId]->FromAbsToPos(abscissa, closestPoint);

    return closestPoint;
}


void Path::ExtractSection(double offset, double abscissa, int curveId, std::shared_ptr<Path>& pathPortion) {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    MoveState(-offset, abscissa, curveId, point);

    pathPortion = std::make_shared<Path>();

    double currentPosition{0};
    while(offset != 0) {
        
        currentPosition = curves_[curveId]->AlongCurveDistance(abscissa);

        double tmp{0};
        double absOffset{offset * curves_[curveId]->EndParameter() / curves_[curveId]->Length()};

        auto curve = curves_[curveId]->ExtractCurveSection(abscissa, absOffset, tmp, offset);
        pathPortion->AddCurveBack<Curve>(curve);

        if(offset > 0) {
            if(curveId + 1 == curvesNumber_)
                offset = 0;
            else {
                ++curveId;
                abscissa = curves_[curveId]->StartParameter();
            }
        }
    }
}


void Path::ExtractSection(double offset, std::shared_ptr<Path>& pathPortion) {

    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    MoveState(-offset, currentAbscissa_, currentCurveId_, point);

    pathPortion = std::make_shared<Path>();

    double currentPosition{0};
    while(offset != 0) {
        
        std::cout << "offset: " << offset << std::endl;
        currentPosition = curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);

        std::cout << "currentPosition: " << currentPosition << ", curves_[currentCurveId_]->Length(): " << curves_[currentCurveId_]->Length() << std::endl;

        double tmp{0};
        double absOffset{offset * curves_[currentCurveId_]->EndParameter() / curves_[currentCurveId_]->Length()};

        std::cout << "currentAbscissa_: " << currentAbscissa_ << std::endl;

        auto curve = curves_[currentCurveId_]->ExtractCurveSection(currentAbscissa_, absOffset, tmp, offset);
        pathPortion->AddCurveBack<Curve>(curve);

        if(offset > 0) {
            std::cout << "Updating currentCurveId_ and currentAbscissa_..." << std::endl;
            if(currentCurveId_ + 1 == curvesNumber_)
                offset = 0;
            else {
                ++currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter();
            }
        }
    }
}

double Path::AlongPathDistance() const {

    double distance{0};
    for(int i = 0; i < currentCurveId_; ++i) {
        distance += curves_[i]->Length();
    }
    distance += curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);

    return distance;
}

void Path::MoveCurrentState(double offset, Eigen::Vector3d& point) {
    
    double currentMetersPosition {curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_)};

    while(offset != 0) {

        if(currentMetersPosition + offset > curves_[currentCurveId_]->Length()){

            if(currentCurveId_ + 1 == curvesNumber_) {
                currentAbscissa_ = curves_[currentCurveId_]->EndParameter();
                curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
                offset = 0;
            }
            else {
                offset -= (curves_[currentCurveId_]->Length() - currentMetersPosition);
                ++currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter();
                currentMetersPosition = curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);
            }
        }
        else if (currentMetersPosition + offset < 0) {

            if(currentCurveId_ == 0) {
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter();
                curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
                offset = 0;
            }
            else {
                offset += currentMetersPosition;
                --currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->EndParameter();
                currentMetersPosition = curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);
            }
        }
        else {

            currentAbscissa_ += offset * curves_[currentCurveId_]->EndParameter() / curves_[currentCurveId_]->Length(); 
            offset = 0;
            curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
        }
    }
}

void Path::MoveState(double offset, double& abscissa, int& curveId, Eigen::Vector3d& point) {
    
    double currentMetersPosition {curves_[curveId]->AlongCurveDistance(abscissa)};

    while(offset != 0) {

        if(currentMetersPosition + offset > curves_[curveId]->Length()){

            if(curveId + 1 == curvesNumber_) {
                abscissa = curves_[curveId]->EndParameter();
                curves_[curveId]->FromAbsToPos(abscissa, point);
                offset = 0;
            }
            else {
                offset -= (curves_[curveId]->Length() - currentMetersPosition);
                ++curveId;
                abscissa = curves_[curveId]->StartParameter();
                currentMetersPosition = curves_[curveId]->AlongCurveDistance(abscissa);
            }
        }
        else if (currentMetersPosition + offset < 0) {
   
            if(curveId == 0) {
                abscissa = curves_[curveId]->StartParameter();
                curves_[curveId]->FromAbsToPos(abscissa, point);
                offset = 0;
            }
            else {
                offset += currentMetersPosition;
                --curveId;
                abscissa = curves_[curveId]->EndParameter();
                currentMetersPosition = curves_[curveId]->AlongCurveDistance(abscissa);
            }
        }
        else {

            abscissa += offset * curves_[curveId]->EndParameter() / curves_[curveId]->Length(); 
            offset = 0;
            curves_[curveId]->FromAbsToPos(abscissa, point);
        }
    }
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