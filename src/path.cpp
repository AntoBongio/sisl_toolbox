#include "sisl_toolbox/path.hpp"

#include <iomanip>

// // Path basato sulle curve definite
// Path::Path(std::vector<Parameters> & parameters)
//     : Path() {

//     for(const auto& elem: parameters) {
//         switch(elem.type){
//             case 0 : // Generic Curve
//             {   
//                 ++curvesNumber_;
//                 curves_.push_back(std::make_shared<GenericCurve>(elem.type, elem.dimension, elem.order, elem.degree, elem.knots, 
//                     elem.points, elem.weights, elem.coefficients));
//                 break;
//             }
//             case 1 : // StraightLine
//             {   
//                 ++curvesNumber_;
//                 curves_.push_back(std::make_shared<StraightLine>(elem.type, elem.dimension, elem.order, elem.startPoint, elem.endPoint));
//                 break;
//             }
//             case 2 : // Circle
//             {
//                 ++curvesNumber_;
//                 //curves_.push_back(std::make_shared<Circle>(elem.type, elem.dimension, elem.order, elem.angle, elem.axis, elem.startPoint, elem.centrePoint));
//                 break;
//             }
//         }
//     }

//     for (const auto & curve: curves_) {
//         length_ += curve->Length();
//     }

//     currentAbscissa_ = curves_[0]->StartParameter_s();
// }

// // Spezzata
// Path::Path(std::vector<Eigen::Vector3d>& points)
//     : curvesNumber_{0} 
//     , length_{0} {

//     for(int i = 0; i < points.size() - 1; ++i) {
//         /*
//         std::cout << "[Broken Line (Path)] -> Segment(" << i << ") -> From [" << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << "] to [" 
//             << points[i+1][0] << ", " << points[i+1][1] << ", " << points[i+1][2] << "]" << std::endl;
//         */
//         curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, points[i], points[i+1]));
//         ++curvesNumber_;
//     }
    
//     if (points.size() > 2) {
//         /*
//         std::cout << "[Broken Line (Path)] -> Segment(" << points.size() - 1 << ") -> From [" << points[points.size() - 1][0] << ", " 
//             << points[points.size() - 1][1] << ", " << points[points.size() - 1][2] << "] to [" 
//             << points[0][0] << ", " << points[0][1] << ", " << points[0][2] << "]" << std::endl;
//         */
//         curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, points[points.size() - 1], points[0]));
//         ++curvesNumber_;
//     }

//     for (const auto & curve: curves_) {
//         length_ += curve->Length();
//     }

//     //std::cout << "[Broken Line (Path)] -> CurvesNumber_: " << curvesNumber_ << ", length: " << length_ << std::endl;
// }

// // Serpentina
// Path::Path(double angle, double offset, std::vector<Eigen::Vector3d>& polygonVerteces)
//     : curvesNumber_{0} 
//     , length_{0} {

//     auto polygon = std::make_shared<Path>(polygonVerteces);
//     polygon->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/polygon.txt");

//     // Compute the rectangle surrounding the polygon
//     double maxX{}; double minX{}; double maxY{}; double minY{};
//     std::tie(maxX, minX, maxY, minY) = this->evalRectangleBoundingBox(polygonVerteces);

//     const Eigen::Vector3d rectangleCentre {maxX + minX / 2, maxY + minY / 2, 0};
//     std::vector<Eigen::Vector3d> rectangleVertices{Eigen::Vector3d{maxX, maxY, 0}, Eigen::Vector3d{maxX, minY, 0},
//                                                    Eigen::Vector3d{minX, minY, 0}, Eigen::Vector3d{minX, maxY, 0}};
//     auto rectangle = std::make_shared<Path>(rectangleVertices);
//     rectangle->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/rectangle.txt"); // Save rectangle

//     // Transform the angle in the interval [0, 360]
//     angle = this->convertToAngleInterval(angle);
//     const double rectangleBase = {(std::abs(maxX) + std::abs(minX)) / 2}; // Eval lenght of the rectangle base
//     const double rectangleHeight = {(std::abs(maxY) + std::abs(minY)) / 2}; // Eval lenght of the rectangle height
//     const double rectangleDiagonal{std::sqrt(std::pow(maxX - minX, 2) + std::pow(maxY - minY, 2))}; // Eval lenght of the rectangle diagonal

//     // Angle in radians
//     const double angleRadians {angle * M_PI / 180.0};


//     /** NOTE: Computation of the Starting Point of the Serpentine */
//     double abscissa{0}; // Not used, needed as output argument for the FindClosestPoint() method
//     int curveId{0}; // Not used, needed as output argument for the FindClosestPoint() method
//     Eigen::Vector3d vertex{0, 0, 0}; // Select a rectangle vertex according to the starting orientation
//     Eigen::Vector3d closestPoint {0, 0, 0}; // According to the starting orientation, eval the polygon closest point to selected vertex of the rectangle

//     auto polygonIntersRectangle = polygon->Intersection(rectangle);
    
//     if(angle <= 90.0) {
//         vertex[0] = maxX;
//         vertex[1] = minY;

//         // Special case: consider as starting value the intersection between polygon and rectangle (right side)
//         if(angle == 90.0) {
//             auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
//                 if(elem[0] == maxX) { return true; } });
//             closestPoint[0] = (*it)[0];
//             closestPoint[1] = (*it)[1];
//         }
//         else 
//             closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
//     }
//     else if (angle <= 180.0) {

//         vertex[0] = maxX;
//         vertex[1] = maxY;

//         // Special case: consider as starting value the intersection between polygon and rectangle (up side)
//         if(angle == 180.0) {
//             auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
//                 if(elem[1] == maxY) { return true; } });
//             closestPoint[0] = (*it)[0];
//             closestPoint[1] = (*it)[1];
//         }
//         else 
//             closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
//     }
//     else if (angle <= 270) {
//         vertex[0] = minX;
//         vertex[1] = maxY;

//         // Special case: consider as starting value the intersection between polygon and rectangle (left side)
//         if(angle == 270.0) {
//             auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
//                 if(elem[0] == minX) { return true; } });
//             closestPoint[0] = (*it)[0];
//             closestPoint[1] = (*it)[1];
//         }
//         else
//             closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
//     }
//     else if (angle <= 360.0 or angle == 0.0){
//         vertex[0] = minX;
//         vertex[1] = minY;

//         // Special case: consider as starting value the intersection between polygon and rectangle (down side)
//         if(angle == 0.0) {
//             auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
//                 if(elem[1] == minY) { return true; } });
//             closestPoint[0] = (*it)[0];
//             closestPoint[1] = (*it)[1];
//         }
//         else 
//             closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
//     }

//     std::cout << "[Serpentine Constructor (Path)] -> ClosestPoint: [" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" << std::endl;

//     // Evalute m, q, delta_q for the set of parallel lines. Consider special cases.
//     double m, q, delta_q;
//     std::vector<Eigen::Vector3d> lineVertex;

//     if(angle == 0 or angle == 180) {
//         m = 0;

//         if(angle == 0) {
//             q = minY;
//             lineVertex.emplace_back(Eigen::Vector3d {minX - rectangleBase, q, 0});
//             lineVertex.emplace_back(Eigen::Vector3d {maxX + rectangleBase, q, 0});
//         }
//         else {
//             q = maxY;
//             lineVertex.emplace_back(Eigen::Vector3d {minX - rectangleBase, q, 0});
//             lineVertex.emplace_back(Eigen::Vector3d {maxX + rectangleBase, q, 0});
//         }
//     }
//     else if(angle  == 90 or angle == 270) {
//         q = 0;
//         if(angle == 90) {
//             m = 1;
//             lineVertex.emplace_back(Eigen::Vector3d {maxX, minY - rectangleHeight, 0});
//             lineVertex.emplace_back(Eigen::Vector3d {maxX, maxY + rectangleHeight, 0});
//         }
//         else {
//             m = -1;
//             lineVertex.emplace_back(Eigen::Vector3d {minX, minY - rectangleHeight, 0});
//             lineVertex.emplace_back(Eigen::Vector3d {minX, maxY + rectangleHeight, 0});
//         }
//     }
//     else {
//         m = std::tan(angleRadians);
//         q = closestPoint[1] - m * closestPoint[0];
//         delta_q = offset / std::cos(angleRadians);
        
//         lineVertex.emplace_back(Eigen::Vector3d { - std::cos(angleRadians) * (rectangleDiagonal), q - std::sin(angleRadians) * (rectangleDiagonal), 0});
//         lineVertex.emplace_back(Eigen::Vector3d {closestPoint[0] + std::cos(angleRadians) * (rectangleDiagonal), closestPoint[1] + std::sin(angleRadians) * (rectangleDiagonal), closestPoint[2]});
//     }

   
//     auto parallelStraightLines = std::make_shared<Path>(); 
    
//     // Add the first line from which all the others will be computed
//     parallelStraightLines->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, lineVertex[0], lineVertex[1]));

//     auto intersectionPoints { parallelStraightLines->Intersection(0, polygon) };
//     intersectionPoints.clear();

//     auto intersecTmp = parallelStraightLines->Intersection(0, polygon);

//     std::map<double, int> map;
//     for(int i = 0; i < intersecTmp.size(); ++i) {
//         map.emplace(this->Distance(intersectionPoints.back(), intersecTmp[i]), i);
//     }
//     for(auto it = map.begin(); it != map.end(); ++it) 
//         intersectionPoints.push_back(intersecTmp[it->second]);
//     map.clear();
   
//     while(true) {

//         if(angle == 0.0 or angle == 180.0) {
//             q = angle == 0 ? (q + offset) : (q - offset);

//             parallelStraightLines->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, 
//                 Eigen::Vector3d{minX - rectangleBase, q, 0} ,
//                 Eigen::Vector3d{maxX + rectangleBase, q, 0}));
//         }
//         else if (angle == 90.0){
//             q += offset;

//             parallelStraightLines->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, 
//                 Eigen::Vector3d{maxX - q, minY - rectangleHeight, 0} ,
//                 Eigen::Vector3d{maxX - q, maxY + rectangleHeight, 0}));
//         } 
//         else if(angle == 270.0) {
//             q += offset;

//             parallelStraightLines->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, 
//                 Eigen::Vector3d{minX + q, minY - rectangleHeight, 0} ,
//                 Eigen::Vector3d{minX + q, maxY + rectangleHeight, 0}));
//         }
//         else {
//             q += delta_q;

//             parallelStraightLines->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, 
//                 Eigen::Vector3d{(minY - 50.0 - q) / m, minY - 50.0, 0} ,
//                 Eigen::Vector3d{(maxY + 50.0 - q) / m, maxY + 50.0, 0}));
//         }

//         auto intersec = parallelStraightLines->Intersection(parallelStraightLines->CurvesNumber() - 1, polygon);

//         for(int i = 0; i < intersec.size(); ++i) {
//             map.emplace(this->Distance(intersectionPoints.back(), intersec[i]), i);
//         }
//         int noMoreThanTwo {0};
//         for(auto it = map.begin(); it != map.end() and noMoreThanTwo < 2; ++it, ++noMoreThanTwo) 
//             intersectionPoints.push_back(intersec[it->second]);
//         map.clear();

//         if(intersec.empty()) 
//             break;
//     }

//     //auto serpentine {std::make_shared<Path>()}; 
//     auto intersec = parallelStraightLines->Intersection(0, polygon);

//     auto previousIntersectionsCounter{intersec.size()};
//     auto intersectionsUpToNow {(intersec.size() >= 2) ? 2 : intersec.size() };

//     auto intersectionsCounter { (intersec.size() >= 2) ? 2 : intersec.size() };
//     Eigen::Vector3d middlePoint;
    
//     // Angles needed to calculate two point of each semi-circunference
//     double angleFirstPointRad {this->convertToAngleInterval(angle + 60.0 - 90.0) * M_PI / 180.0};
//     double angleSecondPointRad {this->convertToAngleInterval(angle + 120.0 - 90.0) * M_PI / 180.0};
//     // Variables used to generate the generic Curve (Semi circunference)
//     std::vector<double> knots {0, 0, 0, 0, 1, 1, 1, 1};
//     std::vector<double> weights {1, 0.33, 0.33, 1};
//     std::vector<double> coefficients {};
//     std::vector<Eigen::Vector3d> circlePoints {};
//     Eigen::Vector3d centreCircle {0, 0, 0};
//     Eigen::Vector3d firstPoint {0, 0, 0};
//     Eigen::Vector3d secondPoint {0, 0, 0};

//     bool noIntersections { false };

//     // Start from the second straight line
//     for(int i = 1; i < parallelStraightLines->CurvesNumber() && !noIntersections; ++i) {

//         // Eval intersections between i-th straight line and the polygon
//         auto intersec = parallelStraightLines->Intersection(i, polygon); 

//         intersectionsUpToNow += (intersec.size() >= 2) ? 2 : intersec.size();
        

//         intersectionsCounter += (intersec.size() >= 2) ? 2 : intersec.size();

//         auto index {intersectionsCounter - 1};

//         if(intersec.size() >= 1) {
//             std::cout << "Iteration: " << i << " -> Case intersec.size() >= 1" << std::endl;

//             double abscissa { 0 };
//             // Take the previous curve and evaluate the closest point w.r.t. the nearest point on the next curve (obtain the abscissa and then generate the point).
//             std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i - 1]->FindClosestPoint(intersectionPoints[index - intersec.size() + 1]);
//             parallelStraightLines->Curves()[i - 1]->FromAbsToPos(abscissa, middlePoint);

//             std::cout << "intersectionPoints[index - intersec.size() + 1]: [" << intersectionPoints[index - intersec.size() + 1][0] << ", " 
//                 << intersectionPoints[index - intersec.size() + 1][1] << ", " << intersectionPoints[index - intersec.size() + 1][2] 
//                 << "], middlePoint: [" << middlePoint[0] << ", " << middlePoint[1] << ", " << middlePoint[2] << "]"
//                 << std::endl;            
            
//             std::shared_ptr<StraightLine> line1;
//             std::shared_ptr<StraightLine> line2;

//             if(previousIntersectionsCounter == 1) {
//                 line1 = std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
//                     middlePoint);
//                 line2 = std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
//                     intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1]);
//             }
//             else {
//                 line1 = std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
//                     middlePoint);
//                 line2 = std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
//                     intersectionPoints[index - intersec.size()]);
//             }

//             if(line1->Length() >= line2->Length()) {
//                 std::cout << "Iteration: " << i << " caso line1->Length() >= line2->Length()" << std::endl;
//                 curves_.emplace_back(line1);
//                 ++curvesNumber_;
//                 //serpentine->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, middlePoint, intersectionPoints[index - 1]));

//                 circlePoints.push_back(middlePoint);

//                 // Do things
//                 centreCircle[0] = (middlePoint[0] + intersectionPoints[intersectionsCounter - intersec.size()][0]) / 2;
//                 centreCircle[1] = (middlePoint[1] + intersectionPoints[intersectionsCounter - intersec.size()][1]) / 2;

//                 firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
//                 firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

//                 secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
//                 secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

//                 auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
//                 directionFirstToSecond /= directionFirstToSecond.norm();
//                 auto lineThroughBoth =  std::make_shared<StraightLine>(1, 3, 3, 
//                     Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * offset,
//                         centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * offset, 0},
//                     Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * offset, 
//                         centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * offset, 0});

//                 auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(curves_[curvesNumber_ - 1]);
//                 std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
//                 if(curvesNumber_ > 1)
//                     lineIntersectSerpentine2 = lineThroughBoth->Intersection(curves_[curvesNumber_ - 2]);

//                 if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

//                     std::cout << "Caso -> Inverti arco di circonferenza" << std::endl;

//                     firstPoint[0] = centreCircle[0] + offset * std::cos(this->convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);
//                     firstPoint[1] = centreCircle[1] + offset * std::sin(this->convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);

//                     secondPoint[0] = centreCircle[0] + offset * std::cos(this->convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
//                     secondPoint[1] = centreCircle[1] + offset * std::sin(this->convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
//                 }

//                 circlePoints.push_back(firstPoint);
//                 circlePoints.push_back(secondPoint);

//                 circlePoints.push_back(intersectionPoints[index - 1]);
//                 curves_.emplace_back(std::make_shared<GenericCurve>(0, 3, 3, 3, knots, circlePoints, weights, coefficients));
//                 ++curvesNumber_;
//             }
//             else {
//                 std::cout << "Iteration: " << i << " caso line1->Length() < line2->Length()" << std::endl;
//                 curves_.emplace_back(line2);
//                 ++curvesNumber_;
//                 std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i]->FindClosestPoint(intersectionPoints[index - intersec.size()]);
//                 parallelStraightLines->Curves()[i]->FromAbsToPos(abscissa, middlePoint);

//                 //serpentine->AddCurveBack(std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - intersec.size()], middlePoint));
//                 circlePoints.push_back(intersectionPoints[index - intersec.size()]);

//                 // Do things
//                 centreCircle[0] = (middlePoint[0] + intersectionPoints[index - intersec.size()][0]) / 2;
//                 centreCircle[1] = (middlePoint[1] + intersectionPoints[index - intersec.size()][1]) / 2;

//                 firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
//                 firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

//                 secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
//                 secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

//                 auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
//                 directionFirstToSecond /= directionFirstToSecond.norm();
//                 auto lineThroughBoth =  std::make_shared<StraightLine>(1, 3, 3, 
//                     Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * offset,
//                         centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * offset, 0},
//                     Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * offset, 
//                         centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * offset, 0});
             

//                 auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(curves_[curvesNumber_ - 1]);
//                 std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
//                 if(curvesNumber_ > 1)
//                     lineIntersectSerpentine2 = lineThroughBoth->Intersection(curves_[curvesNumber_ - 2]);

//                 if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

//                     std::cout << "Caso -> Inverti arco di circonferenza" << std::endl;

//                     firstPoint[0] = centreCircle[0] + offset * std::cos(this->convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);
//                     firstPoint[1] = centreCircle[1] + offset * std::sin(this->convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);

//                     secondPoint[0] = centreCircle[0] + offset * std::cos(this->convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
//                     secondPoint[1] = centreCircle[1] + offset * std::sin(this->convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
//                 }

//                 circlePoints.push_back(firstPoint);
//                 circlePoints.push_back(secondPoint);

//                 circlePoints.push_back(middlePoint);
//                 curves_.emplace_back(std::make_shared<GenericCurve>(0, 3, 3, 3, knots, circlePoints, weights, coefficients));
//                 ++curvesNumber_;
//             }
//         }
//         else {
//             std::cout << "Iteration: " << i << " -> Not anymore intersections!!" << std::endl;

//             curves_.emplace_back(std::make_shared<StraightLine>(1, 3, 3, intersectionPoints[index - 1], intersectionPoints[index]));
//             ++curvesNumber_;
//             noIntersections = true;
//         }

//         circlePoints.clear();
//         previousIntersectionsCounter = intersec.size();
//     }
    
    
//     /*
//     for(int i = 0; i < parallelStraightLines->curvesNumber_; ++i) {
//        std::string path{"/home/antonino/Desktop/sisl_toolbox/script/line" + std::to_string(i) + ".txt"};
//        parallelStraightLines->curves_[i]->SaveCurve(120, path, "write");
//     }
//     */


//     try {
//         std::string const path {"/home/antonino/Desktop/sisl_toolbox/script/startEndPoints.txt"};
//         auto file = std::make_shared<std::ofstream>(path.c_str(), std::ofstream::out);
        
//         *file << (*intersectionPoints.begin())[0] << " " << (*intersectionPoints.begin())[1] << " " << (*intersectionPoints.begin())[2] << "\n";
//         *file << (*(intersectionPoints.end()-1))[0] << " " << (*(intersectionPoints.end()-1))[1] << " " << (*(intersectionPoints.end()-1))[2] << "\n";

//         file->close();

//     } catch (std::exception& e) {
//         std::cerr << "Exception thrown: " << e.what() << std::endl;
//     }

// }


// template <typename T>
// void Path::AddCurveBack(std::shared_ptr<T> curve) {
//     curves_.push_back(curve);
//     //std::cout << "Curve length: " << curve->Length() << std::endl;
//     length_ += curve->Length();
//     ++curvesNumber_; 
// }

// ELIMINARE!!!
// void Path::SavePath(int samples, std::string const path) const {

//     int singleCurveSamples{ static_cast<int>(samples / curvesNumber_) };

//     //std::cout << "samples: " << samples << std::endl;

//     for(int i = 0; i < curvesNumber_; ++i) {
//         if (i == 0)
//             curves_[i]->SaveCurve(singleCurveSamples, path, "write");
//         else
//             curves_[i]->SaveCurve(singleCurveSamples, path, "append");
//     }
// }

// Path vuoto
Path::Path()
: curvesNumber_{0} 
, length_{0} 
, startParameter_m_{0}
, endParameter_m_{0}
, currentAbscissa_ {0} // TOGLIERE
, currentCurveId_{0} {} // TOGLIERE

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
        std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m < startParameter_m_" << std::endl;
        overBound.setLower(abscissa_m, startParameter_m_);
        curveId = 0;
        abscissaCurve_m = curves_[curveId]->StartParameter_m();
    }
    else if(abscissa_m > endParameter_m_) {
        std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m > endParameter_m_" << std::endl;
        overBound.setUpper(abscissa_m, endParameter_m_);
        curveId = curvesNumber_ - 1;
        abscissaCurve_m = curves_[curveId]->EndParameter_m();
    }
    else {
        std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in else" << std::endl;
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

// Se abscissaCurve_m è oltre l'abscissa_m della curva, restituire la abscissa del path nella curva successiva o il limite di questa?
// Implemento il caso che restituisco il limite di questa
std::tuple<double, overBound> Path::CurveAbsToPathAbs(double abscissaCurve_m, int curveId) {

    overBound overBound{};
    double abscissa_m{};
    if(curveId < 0 or curveId > (curvesNumber_ - 1)) {
        std::cout << "CurveId error!!!" << std::endl;
        return std::make_tuple(abscissa_m, overBound);
    }

    if(abscissaCurve_m < curves_[curveId]->StartParameter_m()) {
        std::cout << "[Path::CurveAbsToPathAbs] -> Entrato in abscissaCurve_m < curves_[curveId]->StartParameter_m()" << std::endl;
        overBound.setLower(abscissaCurve_m, curves_[curveId]->StartParameter_m());
        abscissa_m = 0;
    }
    else if (abscissaCurve_m > curves_[curveId]->EndParameter_m()) {
        std::cout << "[Path::CurveAbsToPathAbs] -> Entrato in abscissaCurve_m > curves_[curveId]->EndParameter_m()" << std::endl;
        overBound.setUpper(abscissaCurve_m, curves_[curveId]->EndParameter_m());
        abscissa_m = endParameter_m_;
    }
    else {

    }
    
    return std::make_tuple(abscissa_m, overBound);

    // if(abscissa_m < startParameter_m_) {
    //     std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m < startParameter_m_" << std::endl;
    //     overBound.setLower(abscissa_m, startParameter_m_);
    //     curveId = 0;
    //     abscissaCurve_m = curves_[curveId]->StartParameter_m();
    // }
    // else if(abscissa_m > endParameter_m_) {
    //     std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in abscissa_m > endParameter_m_" << std::endl;
    //     overBound.setUpper(abscissa_m, endParameter_m_);
    //     curveId = curvesNumber_ - 1;
    //     abscissaCurve_m = curves_[curveId]->EndParameter_m();
    // }
    // else {
    //     std::cout << "[Path::PathAbsToCurveAbs] -> Entrato in else" << std::endl;
    //     while(abscissa_m > 0) {
    //         if(abscissa_m > curves_[curveId]->EndParameter_m()) {
    //             abscissa_m -= curves_[curveId]->EndParameter_m();
    //             ++curveId;
    //         }
    //         else {
    //             abscissaCurve_m = abscissa_m;
    //             abscissa_m = 0;
    //         }
    //     }
    // }
    // return std::make_tuple(abscissaCurve_m, curveId, overBound);
}


void Path::MoveState(double offset, double& abscissa, int& curveId, Eigen::Vector3d& point) {
    
    double currentMetersPosition {curves_[curveId]->AlongCurveDistance(abscissa)};

    while(offset != 0) {

        if(currentMetersPosition + offset > curves_[curveId]->Length()){

            if(curveId + 1 == curvesNumber_) {
                abscissa = curves_[curveId]->EndParameter_s();
                curves_[curveId]->FromAbsToPos(abscissa, point);
                offset = 0;
            }
            else {
                offset -= (curves_[curveId]->Length() - currentMetersPosition);
                ++curveId;
                abscissa = curves_[curveId]->StartParameter_s();
                currentMetersPosition = curves_[curveId]->AlongCurveDistance(abscissa);
            }
        }
        else if (currentMetersPosition + offset < 0) {
   
            if(curveId == 0) {
                abscissa = curves_[curveId]->StartParameter_s();
                curves_[curveId]->FromAbsToPos(abscissa, point);
                offset = 0;
            }
            else {
                offset += currentMetersPosition;
                --curveId;
                abscissa = curves_[curveId]->EndParameter_s();
                currentMetersPosition = curves_[curveId]->AlongCurveDistance(abscissa);
            }
        }
        else {

            abscissa += offset * curves_[curveId]->EndParameter_s() / curves_[curveId]->Length(); 
            offset = 0;
            curves_[curveId]->FromAbsToPos(abscissa, point);
        }
    }
}




void Path::ExtractSection(double offset, double abscissa, int curveId, std::shared_ptr<Path>& pathPortion) {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};

    MoveState(-offset, abscissa, curveId, point);

    pathPortion = std::make_shared<Path>();

    double currentPosition{0};
    while(offset != 0) {
        
        currentPosition = curves_[curveId]->AlongCurveDistance(abscissa);

        double tmp{0};
        double absOffset{offset * curves_[curveId]->EndParameter_s() / curves_[curveId]->Length()};

        auto curve = curves_[curveId]->ExtractCurveSection(abscissa, absOffset, tmp, offset);
        pathPortion->AddCurveBack<Curve>(curve);

        if(offset > 0) {
            if(curveId + 1 == curvesNumber_)
                offset = 0;
            else {
                ++curveId;
                abscissa = curves_[curveId]->StartParameter_s();
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
        double absOffset{offset * curves_[currentCurveId_]->EndParameter_s() / curves_[currentCurveId_]->Length()};

        std::cout << "currentAbscissa_: " << currentAbscissa_ << std::endl;

        auto curve = curves_[currentCurveId_]->ExtractCurveSection(currentAbscissa_, absOffset, tmp, offset);
        pathPortion->AddCurveBack<Curve>(curve);

        if(offset > 0) {
            std::cout << "Updating currentCurveId_ and currentAbscissa_..." << std::endl;
            if(currentCurveId_ + 1 == curvesNumber_)
                offset = 0;
            else {
                ++currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter_s();
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
                currentAbscissa_ = curves_[currentCurveId_]->EndParameter_s();
                curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
                offset = 0;
            }
            else {
                offset -= (curves_[currentCurveId_]->Length() - currentMetersPosition);
                ++currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter_s();
                currentMetersPosition = curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);
            }
        }
        else if (currentMetersPosition + offset < 0) {

            if(currentCurveId_ == 0) {
                currentAbscissa_ = curves_[currentCurveId_]->StartParameter_s();
                curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
                offset = 0;
            }
            else {
                offset += currentMetersPosition;
                --currentCurveId_;
                currentAbscissa_ = curves_[currentCurveId_]->EndParameter_s();
                currentMetersPosition = curves_[currentCurveId_]->AlongCurveDistance(currentAbscissa_);
            }
        }
        else {

            currentAbscissa_ += offset * curves_[currentCurveId_]->EndParameter_s() / curves_[currentCurveId_]->Length(); 
            offset = 0;
            curves_[currentCurveId_]->FromAbsToPos(currentAbscissa_, point);
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