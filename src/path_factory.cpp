#include <iostream>
#include "sisl_toolbox/path_factory.hpp"


std::shared_ptr<Path> PathFactory::NewHippodrome(std::vector<Eigen::Vector3d> points) {

        auto hippodrome = std::make_shared<Path>();

        hippodrome->name_ = "Hippodrome";

        try {
            if(points.size() != 4)
                throw points.size();

            // std::cout << "[PathFactory] -> Building a Hippodrome" << std::endl;

            hippodrome->AddCurveBack<StraightLine>(CurveFactory::NewCurve<StraightLine>(points[0], points[1]));

            hippodrome->AddCurveBack<Circle>(CurveFactory::NewCurve<Circle>(3.14, Eigen::Vector3d{0, 0, 1}, points[1], (points[1] + points[2]) / 2));

            hippodrome->AddCurveBack<StraightLine>(CurveFactory::NewCurve<StraightLine>(points[2], points[3]));

            hippodrome->AddCurveBack<Circle>(CurveFactory::NewCurve<Circle>(3.14, Eigen::Vector3d{0, 0, 1}, points[3], (points[3] + points[0]) / 2));
        }
        catch(size_t const & size) {
            throw "[PathFactory] -> Wrong number of points in Hyppodrome Factory! Received a vector of size " + std::to_string(size) + ", while expecting one of size 4.";
        }

        return hippodrome;
    }

std::shared_ptr<Path> PathFactory::NewPolygon(std::vector<Eigen::Vector3d> points) {

        auto polygon = std::make_shared<Path>();

        polygon->name_ = "Polygon";

        try {
            if(points.size() < 2)
                throw points.size();

            // std::cout << "[PathFactory] -> Building a Polygon" << std::endl;

            for(auto it = points.begin(); it != (points.end() - 1); ++it) {
                polygon->AddCurveBack<StraightLine>(CurveFactory::NewCurve<StraightLine>(*it, *(it + 1)));
            }
            polygon->AddCurveBack<StraightLine>(CurveFactory::NewCurve<StraightLine>(*(points.end() - 1), *(points.begin()) ));
        }
        catch(size_t const & size) {
            throw "[PathFactory] -> Wrong number of points in NewPolygon! Received a vector of size " + std::to_string(size) + ", while expecting one of at least size 2.";
        }

        return polygon;
    }

std::shared_ptr<Path> PathFactory::NewSpiral(Eigen::Vector3d centrePoint, Eigen::Vector3d startPoint, double radiusOffset) {

        // std::cout << "[PathFactory] -> Building a Spiral" << std::endl;

        auto spiral = std::make_shared<Path>();

        spiral->name_ = "Spiral";

        double const angle{3.14};
        double radius {Distance(centrePoint, startPoint)};
        auto axis = Eigen::Vector3d{0, 0, 1};

        auto directionCentreToStartPoint = Eigen::Vector3d(centrePoint[0] - startPoint[0], centrePoint[1] - startPoint[1], centrePoint[2] - startPoint[2]);
        directionCentreToStartPoint /= directionCentreToStartPoint.norm();
        directionCentreToStartPoint = -directionCentreToStartPoint;
        spiral->AddCurveBack(CurveFactory::NewCurve<Circle>(angle, axis, startPoint, centrePoint));

        while(radius - radiusOffset > 0) {

            startPoint -= directionCentreToStartPoint * 2 * radius;
            centrePoint -= directionCentreToStartPoint * radiusOffset;
            
            spiral->AddCurveBack(CurveFactory::NewCurve<Circle>(angle, axis, startPoint, centrePoint));
            radius -= radiusOffset;
            directionCentreToStartPoint = -directionCentreToStartPoint;
        }

        return spiral;
    }

std::shared_ptr<Path> PathFactory::NewSerpentine(double angle, double offset, std::vector<Eigen::Vector3d>& polygonVerteces) {

    // std::cout << "[PathFactory] -> Building a Serpentine" << std::endl;

    auto serpentine = std::make_shared<Path>();

    serpentine->name_ = "Serpentine";

    auto polygon = PathFactory::NewPolygon(polygonVerteces);


    /** NOTE: TOGLIERE! */
    PersistenceManager::SaveObj(polygon->Sampling(150), "/home/antonino/Desktop/sisl_toolbox/script/polygon.txt");


    // Compute the rectangle surrounding the polygon
    double maxX{}; double minX{}; double maxY{}; double minY{};
    std::tie(maxX, minX, maxY, minY) = evalRectangleBoundingBox(polygonVerteces);

    const Eigen::Vector3d rectangleCentre {maxX + minX / 2, maxY + minY / 2, 0};
    std::vector<Eigen::Vector3d> rectangleVertices{Eigen::Vector3d{maxX, maxY, 0}, Eigen::Vector3d{maxX, minY, 0},
                                                   Eigen::Vector3d{minX, minY, 0}, Eigen::Vector3d{minX, maxY, 0}};
    auto rectangle = PathFactory::NewPolygon(rectangleVertices);


    /** NOTE: TOGLIERE! */
    PersistenceManager::SaveObj(rectangle->Sampling(150), "/home/antonino/Desktop/sisl_toolbox/script/rectangle.txt");



    // Transform the angle in the interval [0, 360]
    angle = convertToAngleInterval(angle);
    const double rectangleBase = {(std::abs(maxX) + std::abs(minX)) / 2}; // Eval lenght of the rectangle base
    const double rectangleHeight = {(std::abs(maxY) + std::abs(minY)) / 2}; // Eval lenght of the rectangle height
    const double rectangleDiagonal{std::sqrt(std::pow(maxX - minX, 2) + std::pow(maxY - minY, 2))}; // Eval lenght of the rectangle diagonal

    // Angle in radians
    const double angleRadians {angle * M_PI / 180.0};


    /** NOTE: Computation of the Starting Point of the Serpentine */
    double abscissa{0}; // Not used, needed as output argument for the FindClosestPoint() method
    int curveId{0}; // Not used, needed as output argument for the FindClosestPoint() method
    Eigen::Vector3d vertex{0, 0, 0}; // Select a rectangle vertex according to the starting orientation
    Eigen::Vector3d closestPoint {0, 0, 0}; // According to the starting orientation, eval the polygon closest point to selected vertex of the rectangle

    auto polygonIntersRectangle = polygon->Intersection(rectangle);
    
    if(angle <= 90.0) {
        vertex[0] = maxX;
        vertex[1] = minY;

        // Special case: consider as starting value the intersection between polygon and rectangle (right side)
        if(angle == 90.0) {
            auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
                if(elem[0] == maxX) { return true; } });
            closestPoint[0] = (*it)[0];
            closestPoint[1] = (*it)[1];
        }
        else 
            closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
    }
    else if (angle <= 180.0) {

        vertex[0] = maxX;
        vertex[1] = maxY;

        // Special case: consider as starting value the intersection between polygon and rectangle (up side)
        if(angle == 180.0) {
            auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
                if(elem[1] == maxY) { return true; } });
            closestPoint[0] = (*it)[0];
            closestPoint[1] = (*it)[1];
        }
        else 
            closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
    }
    else if (angle <= 270) {
        vertex[0] = minX;
        vertex[1] = maxY;

        // Special case: consider as starting value the intersection between polygon and rectangle (left side)
        if(angle == 270.0) {
            auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
                if(elem[0] == minX) { return true; } });
            closestPoint[0] = (*it)[0];
            closestPoint[1] = (*it)[1];
        }
        else
            closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
    }
    else if (angle <= 360.0 or angle == 0.0){
        vertex[0] = minX;
        vertex[1] = minY;

        // Special case: consider as starting value the intersection between polygon and rectangle (down side)
        if(angle == 0.0) {
            auto it = std::find_if(polygonIntersRectangle.begin(), polygonIntersRectangle.end(), [&](auto const & elem) mutable {
                if(elem[1] == minY) { return true; } });
            closestPoint[0] = (*it)[0];
            closestPoint[1] = (*it)[1];
        }
        else 
            closestPoint = polygon->FindClosestPoint(vertex, curveId, abscissa);
    }

    // Evalute m, q, delta_q for the set of parallel lines. Consider special cases.
    double m, q, delta_q;
    std::vector<Eigen::Vector3d> lineVertex;

    if(angle == 0 or angle == 180) {
        m = 0;

        if(angle == 0) {
            q = minY;
            lineVertex.emplace_back(Eigen::Vector3d {minX - rectangleBase, q, 0});
            lineVertex.emplace_back(Eigen::Vector3d {maxX + rectangleBase, q, 0});
        }
        else {
            q = maxY;
            lineVertex.emplace_back(Eigen::Vector3d {minX - rectangleBase, q, 0});
            lineVertex.emplace_back(Eigen::Vector3d {maxX + rectangleBase, q, 0});
        }
    }
    else if(angle  == 90 or angle == 270) {
        q = 0;
        if(angle == 90) {
            m = 1;
            lineVertex.emplace_back(Eigen::Vector3d {maxX, minY - rectangleHeight, 0});
            lineVertex.emplace_back(Eigen::Vector3d {maxX, maxY + rectangleHeight, 0});
        }
        else {
            m = -1;
            lineVertex.emplace_back(Eigen::Vector3d {minX, minY - rectangleHeight, 0});
            lineVertex.emplace_back(Eigen::Vector3d {minX, maxY + rectangleHeight, 0});
        }
    }
    else {
        m = std::tan(angleRadians);
        q = closestPoint[1] - m * closestPoint[0];
        delta_q = offset / std::cos(angleRadians);
        
        lineVertex.emplace_back(Eigen::Vector3d { - std::cos(angleRadians) * (rectangleDiagonal), q - std::sin(angleRadians) * (rectangleDiagonal), 0});
        lineVertex.emplace_back(Eigen::Vector3d {closestPoint[0] + std::cos(angleRadians) * (rectangleDiagonal), closestPoint[1] + std::sin(angleRadians) * (rectangleDiagonal), closestPoint[2]});
    }

   
    auto parallelStraightLines = std::make_shared<Path>(); 

    // Add the first line from which all the others will be computed
    parallelStraightLines->AddCurveBack(CurveFactory::NewCurve<StraightLine>(lineVertex[0], lineVertex[1]));

    auto intersectionPoints { parallelStraightLines->Intersection(0, polygon) };
    intersectionPoints.clear();

    auto intersecTmp = parallelStraightLines->Intersection(0, polygon);

    std::map<double, int> map;
    for(int i = 0; i < intersecTmp.size(); ++i) {
        map.emplace(Distance(intersectionPoints.back(), intersecTmp[i]), i);
    }
    for(auto it = map.begin(); it != map.end(); ++it) 
        intersectionPoints.push_back(intersecTmp[it->second]);
    map.clear();
   
    while(true) {

        if(angle == 0.0 or angle == 180.0) {
            q = angle == 0 ? (q + offset) : (q - offset);

            parallelStraightLines->AddCurveBack(CurveFactory::NewCurve<StraightLine>(
                Eigen::Vector3d{minX - rectangleBase, q, 0} ,
                Eigen::Vector3d{maxX + rectangleBase, q, 0}));
        }
        else if (angle == 90.0){
            q += offset;

            parallelStraightLines->AddCurveBack(CurveFactory::NewCurve<StraightLine>(
                Eigen::Vector3d{maxX - q, minY - rectangleHeight, 0} ,
                Eigen::Vector3d{maxX - q, maxY + rectangleHeight, 0}));
        } 
        else if(angle == 270.0) {
            q += offset;

            parallelStraightLines->AddCurveBack(CurveFactory::NewCurve<StraightLine>(
                Eigen::Vector3d{minX + q, minY - rectangleHeight, 0} ,
                Eigen::Vector3d{minX + q, maxY + rectangleHeight, 0}));
        }
        else {
            q += delta_q;

            parallelStraightLines->AddCurveBack(CurveFactory::NewCurve<StraightLine>(
                Eigen::Vector3d{(minY - 50.0 - q) / m, minY - 50.0, 0} ,
                Eigen::Vector3d{(maxY + 50.0 - q) / m, maxY + 50.0, 0}));
        }

        auto intersec = parallelStraightLines->Intersection(parallelStraightLines->CurvesNumber() - 1, polygon);

        for(int i = 0; i < intersec.size(); ++i) {
            map.emplace(Distance(intersectionPoints.back(), intersec[i]), i);
        }
        int noMoreThanTwo {0};
        for(auto it = map.begin(); it != map.end() and noMoreThanTwo < 2; ++it, ++noMoreThanTwo) 
            intersectionPoints.push_back(intersec[it->second]);
        map.clear();

        if(intersec.empty()) 
            break;
    }

    auto intersec = parallelStraightLines->Intersection(0, polygon);

    auto previousIntersectionsCounter{intersec.size()};
    auto intersectionsUpToNow {(intersec.size() >= 2) ? 2 : intersec.size() };

    auto intersectionsCounter { (intersec.size() >= 2) ? 2 : intersec.size() };
    Eigen::Vector3d middlePoint;
    
    // Angles needed to calculate two point of each semi-circunference
    double angleFirstPointRad {convertToAngleInterval(angle + 60.0 - 90.0) * M_PI / 180.0};
    double angleSecondPointRad {convertToAngleInterval(angle + 120.0 - 90.0) * M_PI / 180.0};
    // Variables used to generate the generic Curve (Semi circunference)
    std::vector<double> knots {0, 0, 0, 0, 1, 1, 1, 1};
    std::vector<double> weights {1, 0.33, 0.33, 1};
    std::vector<double> coefficients {};
    std::vector<Eigen::Vector3d> circlePoints {};
    Eigen::Vector3d centreCircle {0, 0, 0};
    Eigen::Vector3d firstPoint {0, 0, 0};
    Eigen::Vector3d secondPoint {0, 0, 0};

    bool noIntersections { false };

    // Start from the second straight line
    for(int i = 1; i < parallelStraightLines->CurvesNumber() && !noIntersections; ++i) {

        // Eval intersections between i-th straight line and the polygon
        auto intersec = parallelStraightLines->Intersection(i, polygon); 

        intersectionsUpToNow += (intersec.size() >= 2) ? 2 : intersec.size();
        

        intersectionsCounter += (intersec.size() >= 2) ? 2 : intersec.size();

        auto index {intersectionsCounter - 1};

        if(intersec.size() >= 1) {
            // std::cout << "Iteration: " << i << " -> Case intersec.size() >= 1" << std::endl;

            double abscissa { 0 };
            // Take the previous curve and evaluate the closest point w.r.t. the nearest point on the next curve (obtain the abscissa and then generate the point).
            std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i - 1]->FindClosestPoint(intersectionPoints[index - intersec.size() + 1]);
            middlePoint = parallelStraightLines->Curves()[i - 1]->At(abscissa);

            // std::cout << "intersectionPoints[index - intersec.size() + 1]: [" << intersectionPoints[index - intersec.size() + 1][0] << ", " 
            //     << intersectionPoints[index - intersec.size() + 1][1] << ", " << intersectionPoints[index - intersec.size() + 1][2] 
            //     << "], middlePoint: [" << middlePoint[0] << ", " << middlePoint[1] << ", " << middlePoint[2] << "]"
            //     << std::endl;            
            
            std::shared_ptr<StraightLine> line1;
            std::shared_ptr<StraightLine> line2;

            line1 = CurveFactory::NewCurve<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    middlePoint);

            if(previousIntersectionsCounter == 1) {
                line2 = CurveFactory::NewCurve<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1]);
            }
            else {
                line2 = CurveFactory::NewCurve<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size()]);
            }

            if(line1->Length() >= line2->Length()) {
                // std::cout << "Iteration: " << i << " caso line1->Length() >= line2->Length()" << std::endl;
                serpentine->AddCurveBack(line1);

                double angleTest{3.14};

                circlePoints.push_back(middlePoint);

                // Do things
                centreCircle[0] = (middlePoint[0] + intersectionPoints[intersectionsCounter - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[intersectionsCounter - intersec.size()][1]) / 2;

                firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
                firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

                secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
                secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();
                auto lineThroughBoth =  CurveFactory::NewCurve<StraightLine>(
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * offset,
                        centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * offset, 0},
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * offset, 
                        centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * offset, 0});

                auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(serpentine->Curves()[serpentine->CurvesNumber() - 1]);
                std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
                if(serpentine->CurvesNumber() > 1)
                    lineIntersectSerpentine2 = lineThroughBoth->Intersection(serpentine->Curves()[serpentine->CurvesNumber() - 2]);

                if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

                    angleTest = - angleTest;

                    // std::cout << "Caso -> Inverti arco di circonferenza" << std::endl;

                    // firstPoint[0] = centreCircle[0] + offset * std::cos(convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);
                    // firstPoint[1] = centreCircle[1] + offset * std::sin(convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);

                    // secondPoint[0] = centreCircle[0] + offset * std::cos(convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
                    // secondPoint[1] = centreCircle[1] + offset * std::sin(convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
                }

                // circlePoints.push_back(firstPoint);
                // circlePoints.push_back(secondPoint);

                // circlePoints.push_back(intersectionPoints[index - 1]);

                serpentine->AddCurveBack(CurveFactory::NewCurve<Circle>(angleTest, Eigen::Vector3d{0, 0, 1}, middlePoint, centreCircle));

                //serpentine->AddCurveBack(CurveFactory::NewCurve<GenericCurve>(3, knots, circlePoints, weights, coefficients));

            }
            else {
                // std::cout << "Iteration: " << i << " caso line1->Length() < line2->Length()" << std::endl;

                serpentine->AddCurveBack(line2);

                double angleTest {3.14};

                std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i]->FindClosestPoint(intersectionPoints[index - intersec.size()]);
                middlePoint = parallelStraightLines->Curves()[i]->At(abscissa);

                circlePoints.push_back(intersectionPoints[index - intersec.size()]);

                // Do things
                centreCircle[0] = (middlePoint[0] + intersectionPoints[index - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[index - intersec.size()][1]) / 2;

                firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
                firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

                secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
                secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();
                auto lineThroughBoth =  CurveFactory::NewCurve<StraightLine>(
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * offset,
                        centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * offset, 0},
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * offset, 
                        centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * offset, 0});
             

                auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(serpentine->Curves()[serpentine->CurvesNumber() - 1]);
                std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
                if(serpentine->CurvesNumber() > 1)
                    lineIntersectSerpentine2 = lineThroughBoth->Intersection(serpentine->Curves()[serpentine->CurvesNumber() - 2]);

                if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

                    angleTest = - angleTest;

                    // std::cout << "Caso -> Inverti arco di circonferenza" << std::endl;

                    // firstPoint[0] = centreCircle[0] + offset * std::cos(convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);
                    // firstPoint[1] = centreCircle[1] + offset * std::sin(convertToAngleInterval(angle - 60.0 - 90.0) * M_PI / 180.0);

                    // secondPoint[0] = centreCircle[0] + offset * std::cos(convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
                    // secondPoint[1] = centreCircle[1] + offset * std::sin(convertToAngleInterval(angle - 120.0 - 90.0) * M_PI / 180.0);
                }

                // circlePoints.push_back(firstPoint);
                // circlePoints.push_back(secondPoint);

                // circlePoints.push_back(middlePoint);

                serpentine->AddCurveBack(CurveFactory::NewCurve<Circle>(angleTest, Eigen::Vector3d{0, 0, 1}, intersectionPoints[index - intersec.size()], centreCircle));

                //serpentine->AddCurveBack(CurveFactory::NewCurve<GenericCurve>(3, knots, circlePoints, weights, coefficients));

            }
        }
        else {
            // std::cout << "Iteration: " << i << " -> Not anymore intersections!!" << std::endl;

            serpentine->AddCurveBack(CurveFactory::NewCurve<StraightLine>(intersectionPoints[index - 1], intersectionPoints[index]));
            noIntersections = true;
        }

        circlePoints.clear();
        previousIntersectionsCounter = intersec.size();
    }
    
    
    try {
        std::string const path {"/home/antonino/Desktop/sisl_toolbox/script/startEndPoints.txt"};
        auto file = std::make_shared<std::ofstream>(path.c_str(), std::ofstream::out);
        
        *file << (*intersectionPoints.begin())[0] << " " << (*intersectionPoints.begin())[1] << " " << (*intersectionPoints.begin())[2] << "\n";
        *file << (*(intersectionPoints.end()-1))[0] << " " << (*(intersectionPoints.end()-1))[1] << " " << (*(intersectionPoints.end()-1))[2] << "\n";

        file->close();

    } catch (std::exception& e) {
        std::cerr << "Exception thrown: " << e.what() << std::endl;
    }

    return serpentine;

}
