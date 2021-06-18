#include <iostream>
#include "sisl_toolbox/path_factory.hpp"

#include <sisl_toolbox/path.hpp>
#include "sisl_toolbox/straight_line.hpp"
#include "sisl_toolbox/circular_arc.hpp"
#include "sisl_toolbox/generic_curve.hpp"


std::shared_ptr<Path> PathFactory::NewHippodrome(std::vector<Eigen::Vector3d> points) {

        auto hippodrome = std::make_shared<Path>();

        hippodrome->name_ = "Hippodrome";

        try {
            if(points.size() != 4)
                throw points.size();

            // std::cout << "[PathFactory] -> Building a Hippodrome" << std::endl;

            hippodrome->AddCurveBack(std::make_shared<StraightLine>(points[0], points[1]));

            hippodrome->AddCurveBack(std::make_shared<CircularArc>(3.14, Eigen::Vector3d{0, 0, 1}, points[1], (points[1] + points[2]) / 2));

            hippodrome->AddCurveBack(std::make_shared<StraightLine>(points[2], points[3]));

            hippodrome->AddCurveBack(std::make_shared<CircularArc>(3.14, Eigen::Vector3d{0, 0, 1}, points[3], (points[3] + points[0]) / 2));
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
                polygon->AddCurveBack(std::make_shared<StraightLine>(*it, *(it + 1)));
            }
            polygon->AddCurveBack(std::make_shared<StraightLine>(*(points.end() - 1), *(points.begin()) ));
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
        spiral->AddCurveBack(std::make_shared<CircularArc>(angle, axis, startPoint, centrePoint));

        while(radius - radiusOffset > 0) {

            startPoint -= directionCentreToStartPoint * 2 * radius;
            centrePoint -= directionCentreToStartPoint * radiusOffset;
            
            spiral->AddCurveBack(std::make_shared<CircularArc>(angle, axis, startPoint, centrePoint));
            radius -= radiusOffset;
            directionCentreToStartPoint = -directionCentreToStartPoint;
        }

        return spiral;
    }

std::shared_ptr<Path> PathFactory::NewSerpentine(double angle, int direction, double offset, std::vector<Eigen::Vector3d>& polygonVerteces) {

    auto serpentine = std::make_shared<Path>();

    serpentine->name_ = "Serpentine";

    auto polygon = PathFactory::NewPolygon(polygonVerteces);

    // Compute the rectangle surrounding the polygon
    double maxX{}; double minX{}; double maxY{}; double minY{};
    std::tie(maxX, minX, maxY, minY) = evalRectangleBoundingBox(polygonVerteces);

    const Eigen::Vector3d rectangleCentre {(maxX + minX) / 2, (maxY + minY) / 2, 0};
    std::vector<Eigen::Vector3d> rectangleVertices{Eigen::Vector3d{maxX, maxY, 0}, Eigen::Vector3d{maxX, minY, 0},
                                                   Eigen::Vector3d{minX, minY, 0}, Eigen::Vector3d{minX, maxY, 0}};
    auto rectangle = PathFactory::NewPolygon(rectangleVertices);

    // Transform the angle in the interval [0, 360)
    angle = convertToAngleInterval(angle);
    const double angleRadians {DegToRad(angle)};
    const double rectangleDiagonal{std::sqrt(std::pow(maxX - minX, 2) + std::pow(maxY - minY, 2))}; // Eval lenght of the rectangle diagonal


    /** NOTE: Computation of the Starting Point of the Serpentine */
    auto polygonIntersRectangle = polygon->Intersection(rectangle);
    auto firstLine = std::make_shared<StraightLine>(
        Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians), 
                        rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians), 0}, 
        Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians), 
                        rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians), 0});
    auto nextLine = firstLine;

    double shiftingValue{2}; // Precision
    int counter{2};

    while(polygon->Intersection(nextLine).size() > 0) {

        firstLine = nextLine;

        if(direction == RIGHT) {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians) - counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians) - counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians) - counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians) - counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0});
        }
        else {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians) + counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians) + counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians) + counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians) + counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0});
        }
        ++counter;
    }

    Eigen::Vector3d startPointSegment {firstLine->StartPoint()};
    Eigen::Vector3d endPointSegment {firstLine->EndPoint()};

    auto parallelStraightLines = std::make_shared<Path>();

    counter = 1;
    nextLine = firstLine;

    std::vector<Eigen::Vector3d> intersectionPoints{};
    bool changeDirection {false};

    while(polygon->Intersection(nextLine).size() > 0) {
        
        parallelStraightLines->AddCurveBack(nextLine);

        auto intersecTmp = polygon->Intersection(parallelStraightLines->LastCurve());
        if(intersecTmp.size() == 1) {
            intersectionPoints.push_back(intersecTmp[0]);
        } 
        else {
            double angleDirection = std::atan2(intersecTmp[1][1] - intersecTmp[0][1], intersecTmp[1][0] - intersecTmp[0][0]);
            if(angleDirection < 0) angleDirection += 2*M_PI;
            angleDirection = std::round(RadToDeg(angleDirection));

            if(angleDirection == angle) {
                if(changeDirection) {
                    intersectionPoints.push_back(intersecTmp[1]);
                    intersectionPoints.push_back(intersecTmp[0]);
                } 
                else {
                    intersectionPoints.push_back(intersecTmp[0]);
                    intersectionPoints.push_back(intersecTmp[1]);
                }
            } 
            else {
                if(changeDirection) {
                    intersectionPoints.push_back(intersecTmp[0]);
                    intersectionPoints.push_back(intersecTmp[1]);
                } 
                else {
                    intersectionPoints.push_back(intersecTmp[1]);
                    intersectionPoints.push_back(intersecTmp[0]);
                }
            }
            changeDirection = !changeDirection;
        }
        
        if(direction == RIGHT) {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{startPointSegment[0] + counter * offset * std::cos(angleRadians + M_PI/2), 
                                startPointSegment[1] + counter * offset * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{endPointSegment[0] + counter * offset * std::cos(angleRadians + M_PI/2), 
                                endPointSegment[1] + counter * offset * std::sin(angleRadians + M_PI/2), 0});
        }
        else {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{startPointSegment[0] - counter * offset * std::cos(angleRadians + M_PI/2), 
                                startPointSegment[1] - counter * offset * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{endPointSegment[0] - counter * offset * std::cos(angleRadians + M_PI/2), 
                                endPointSegment[1] - counter * offset * std::sin(angleRadians + M_PI/2), 0});
        }

        ++counter;
    }

    parallelStraightLines->AddCurveBack(nextLine);

    auto intersec = parallelStraightLines->Intersection(0, polygon);
    auto previousIntersectionsCounter{intersec.size()};
    auto intersectionsCounter { (intersec.size() >= 2) ? 2 : intersec.size() };
    Eigen::Vector3d middlePoint;
    
    // Angles needed to calculate two point of each semi-circunference

    double angleFirstPointRad{};
    double angleSecondPointRad{};

    if(direction == RIGHT) {
        angleFirstPointRad = convertToAngleInterval(angle + 60.0 - 90.0) * M_PI / 180.0;
        angleSecondPointRad = convertToAngleInterval(angle + 120.0 - 90.0) * M_PI / 180.0;
    }
    else {
        angleFirstPointRad = convertToAngleInterval(angle + 60.0 + 90.0) * M_PI / 180.0;
        angleSecondPointRad = convertToAngleInterval(angle + 120.0 + 90.0) * M_PI / 180.0;
    }

    std::vector<Eigen::Vector3d> circlePoints {};
    Eigen::Vector3d centreCircle {0, 0, 0};
    Eigen::Vector3d firstPoint {0, 0, 0};
    Eigen::Vector3d secondPoint {0, 0, 0};

    bool noIntersections { false };

    // Start from the second straight line
    for(int i = 1; i < parallelStraightLines->CurvesNumber() && !noIntersections; ++i) {

        // Eval intersections between i-th straight line and the polygon
        auto intersec = parallelStraightLines->Intersection(i, polygon);         

        intersectionsCounter += (intersec.size() >= 2) ? 2 : intersec.size();

        auto index {intersectionsCounter - 1};
        
        std::shared_ptr<StraightLine> line1;
        std::shared_ptr<StraightLine> line2;

        if(intersec.size() >= 1) {

            double abscissa { 0 };
            // Take the previous curve and evaluate the closest point w.r.t. the nearest point on the next curve (obtain the abscissa and then generate the point).
            std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i - 1]->FindClosestPoint(intersectionPoints[index - intersec.size() + 1]);
            middlePoint = parallelStraightLines->Curves()[i - 1]->At(abscissa);

            

            line1 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    middlePoint);

            if(previousIntersectionsCounter == 1) {
                line2 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1]);
            }
            else {
                line2 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size()]);
            }

            if(line1->Length() >= line2->Length()) {
                serpentine->AddCurveBack(line1);

                double angleTest{3.14};

                circlePoints.push_back(middlePoint);

                centreCircle[0] = (middlePoint[0] + intersectionPoints[intersectionsCounter - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[intersectionsCounter - intersec.size()][1]) / 2;

                firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
                firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

                secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
                secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();
                auto lineThroughBoth =  std::make_shared<StraightLine>(
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
                }

                serpentine->AddCurveBack(std::make_shared<CircularArc>(angleTest, Eigen::Vector3d{0, 0, 1}, middlePoint, centreCircle));
            }
            else {

                serpentine->AddCurveBack(line2);

                double angleTest {3.14};

                std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i]->FindClosestPoint(intersectionPoints[index - intersec.size()]);
                middlePoint = parallelStraightLines->Curves()[i]->At(abscissa);

                circlePoints.push_back(intersectionPoints[index - intersec.size()]);

                centreCircle[0] = (middlePoint[0] + intersectionPoints[index - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[index - intersec.size()][1]) / 2;

                firstPoint[0] = centreCircle[0] + offset * std::cos(angleFirstPointRad);
                firstPoint[1] = centreCircle[1] + offset * std::sin(angleFirstPointRad);

                secondPoint[0] = centreCircle[0] + offset * std::cos(angleSecondPointRad);
                secondPoint[1] = centreCircle[1] + offset * std::sin(angleSecondPointRad);

                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();
                auto lineThroughBoth =  std::make_shared<StraightLine>(
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
                }

                serpentine->AddCurveBack(std::make_shared<CircularArc>(angleTest, Eigen::Vector3d{0, 0, 1}, intersectionPoints[index - intersec.size()], centreCircle));
            }
        }
        else {

            serpentine->AddCurveBack(std::make_shared<StraightLine>(intersectionPoints[index - 1], intersectionPoints[index]));
            noIntersections = true;
        }

        circlePoints.clear();
        previousIntersectionsCounter = intersec.size();
    }
    
    return serpentine;
}



std::shared_ptr<Path> PathFactory::NewRaceTrack(double angle, int direction, double firstRadius, double secondRadius, 
    std::vector<Eigen::Vector3d>& polygonVerteces) {

    if(firstRadius < secondRadius)
        std::swap(firstRadius, secondRadius);

    auto raceTrack = std::make_shared<Path>();

    raceTrack->name_ = "Race Track";

    auto polygon = PathFactory::NewPolygon(polygonVerteces);

    // Compute the rectangle surrounding the polygon
    double maxX{}; double minX{}; double maxY{}; double minY{};
    std::tie(maxX, minX, maxY, minY) = evalRectangleBoundingBox(polygonVerteces);

    const Eigen::Vector3d rectangleCentre {(maxX + minX) / 2, (maxY + minY) / 2, 0};
    std::vector<Eigen::Vector3d> rectangleVertices{Eigen::Vector3d{maxX, maxY, 0}, Eigen::Vector3d{maxX, minY, 0},
                                                   Eigen::Vector3d{minX, minY, 0}, Eigen::Vector3d{minX, maxY, 0}};
    auto rectangle = PathFactory::NewPolygon(rectangleVertices);

    // Transform the angle in the interval [0, 360)
    angle = convertToAngleInterval(angle);
    const double rectangleBase = {(std::abs(maxY) + std::abs(minY)) / 2}; // Eval lenght of the rectangle base
    const double rectangleHeight = {(std::abs(maxX) + std::abs(minX)) / 2}; // Eval lenght of the rectangle height
    const double rectangleDiagonal{std::sqrt(std::pow(maxX - minX, 2) + std::pow(maxY - minY, 2))}; // Eval lenght of the rectangle diagonal

    // Angle in radians
    const double angleRadians {angle * M_PI / 180.0};


    /** NOTE: Computation of the Starting Point of the Serpentine */
    
    auto polygonIntersRectangle = polygon->Intersection(rectangle);
    auto firstLine = std::make_shared<StraightLine>(
        Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians), 
                        rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians), 0}, 
        Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians), 
                        rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians), 0});
    auto nextLine = firstLine;

    double shiftingValue{2};
    int counter{2};

    // Logica caso RIGHT
    while(polygon->Intersection(nextLine).size() > 0) {

        firstLine = nextLine;

        if(direction == RIGHT) {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians) - counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians) - counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians) - counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians) - counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0});
        }
        else {
            nextLine = std::make_shared<StraightLine>(
                Eigen::Vector3d{rectangleCentre[0] - 2 * rectangleDiagonal * std::cos(angleRadians) + counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] - 2 * rectangleDiagonal * std::sin(angleRadians) + counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0}, 
                Eigen::Vector3d{rectangleCentre[0] + 2 * rectangleDiagonal * std::cos(angleRadians) + counter * shiftingValue * std::cos(angleRadians + M_PI/2), 
                                rectangleCentre[1] + 2 * rectangleDiagonal * std::sin(angleRadians) + counter * shiftingValue * std::sin(angleRadians + M_PI/2), 0});
        }
        ++counter;
    }

    Eigen::Vector3d startPointSegment {firstLine->StartPoint()};
    Eigen::Vector3d endPointSegment {firstLine->EndPoint()};

    auto parallelStraightLines = std::make_shared<Path>();

    counter = 1;
    nextLine = firstLine;

    std::vector<Eigen::Vector3d> intersectionPoints{};
    bool changeDirection {false};
    bool changeRadius{false};

    while(polygon->Intersection(nextLine).size() > 0) {
        
        parallelStraightLines->AddCurveBack(nextLine);

        auto intersecTmp = polygon->Intersection(parallelStraightLines->LastCurve());
        if(intersecTmp.size() == 1) {
            intersectionPoints.push_back(intersecTmp[0]);
        } 
        else {
            double angleDirection = std::atan2(intersecTmp[1][1] - intersecTmp[0][1], intersecTmp[1][0] - intersecTmp[0][0]);
            if(angleDirection < 0) angleDirection += 2*M_PI;
            angleDirection = std::round(RadToDeg(angleDirection));

            if(angleDirection == angle) {
                if(changeDirection) {
                    intersectionPoints.push_back(intersecTmp[1]);
                    intersectionPoints.push_back(intersecTmp[0]);
                } 
                else {
                    intersectionPoints.push_back(intersecTmp[0]);
                    intersectionPoints.push_back(intersecTmp[1]);
                }
            } 
            else {
                if(changeDirection) {
                    intersectionPoints.push_back(intersecTmp[0]);
                    intersectionPoints.push_back(intersecTmp[1]);
                } 
                else {
                    intersectionPoints.push_back(intersecTmp[1]);
                    intersectionPoints.push_back(intersecTmp[0]);
                }
            }
            changeDirection = !changeDirection;
        }

        
        
        if(direction == RIGHT) {
            double counter = 1;
            if(changeRadius) {

                

                startPointSegment[0] = startPointSegment[0] - counter * secondRadius * std::cos(angleRadians + M_PI/2);
                startPointSegment[1] = startPointSegment[1] - counter * secondRadius * std::sin(angleRadians + M_PI/2);
                endPointSegment[0] = endPointSegment[0] - counter * secondRadius * std::cos(angleRadians + M_PI/2);
                endPointSegment[1] = endPointSegment[1] - counter * secondRadius * std::sin(angleRadians + M_PI/2);

                nextLine = std::make_shared<StraightLine>(
                    Eigen::Vector3d{startPointSegment[0], startPointSegment[1], 0}, 
                    Eigen::Vector3d{endPointSegment[0], endPointSegment[1], 0});
            }
            else {
                startPointSegment[0] = startPointSegment[0] + counter * firstRadius * std::cos(angleRadians + M_PI/2);
                startPointSegment[1] = startPointSegment[1] + counter * firstRadius * std::sin(angleRadians + M_PI/2);
                endPointSegment[0] = endPointSegment[0] + counter * firstRadius * std::cos(angleRadians + M_PI/2);
                endPointSegment[1] = endPointSegment[1] + counter * firstRadius * std::sin(angleRadians + M_PI/2);

                nextLine = std::make_shared<StraightLine>(
                    Eigen::Vector3d{startPointSegment[0], startPointSegment[1], 0}, 
                    Eigen::Vector3d{endPointSegment[0], endPointSegment[1], 0});
            }
        }
        else {
            double counter = 1;
            if(changeRadius) {
                // std::cout << "Using secondRadius..." << std::endl;

                startPointSegment[0] = startPointSegment[0] + counter * secondRadius * std::cos(angleRadians + M_PI/2);
                startPointSegment[1] = startPointSegment[1] + counter * secondRadius * std::sin(angleRadians + M_PI/2);
                endPointSegment[0] = endPointSegment[0] + counter * secondRadius * std::cos(angleRadians + M_PI/2);
                endPointSegment[1] = endPointSegment[1] + counter * secondRadius * std::sin(angleRadians + M_PI/2);

                nextLine = std::make_shared<StraightLine>(
                    Eigen::Vector3d{startPointSegment[0], startPointSegment[1], 0}, 
                    Eigen::Vector3d{endPointSegment[0], endPointSegment[1], 0});
            }
            else {
                // std::cout << "Using firstRadius..." << std::endl;

                startPointSegment[0] = startPointSegment[0] - counter * firstRadius * std::cos(angleRadians + M_PI/2);
                startPointSegment[1] = startPointSegment[1] - counter * firstRadius * std::sin(angleRadians + M_PI/2);
                endPointSegment[0] = endPointSegment[0] - counter * firstRadius * std::cos(angleRadians + M_PI/2);
                endPointSegment[1] = endPointSegment[1] - counter * firstRadius * std::sin(angleRadians + M_PI/2);

                nextLine = std::make_shared<StraightLine>(
                    Eigen::Vector3d{startPointSegment[0], startPointSegment[1], 0}, 
                    Eigen::Vector3d{endPointSegment[0], endPointSegment[1], 0});
            }
        }

        changeRadius = !changeRadius;

        ++counter;
    }

    parallelStraightLines->AddCurveBack(nextLine);

       auto intersec = parallelStraightLines->Intersection(0, polygon);

    auto previousIntersectionsCounter{intersec.size()};
    auto intersectionsCounter { (intersec.size() >= 2) ? 2 : intersec.size() };
    Eigen::Vector3d middlePoint;
    
    // Angles needed to calculate two point of each semi-circunference

    double angleFirstPointRad{};
    double angleSecondPointRad{};

    if(direction == RIGHT) {
        angleFirstPointRad = convertToAngleInterval(angle + 60.0 - 90.0) * M_PI / 180.0;
        angleSecondPointRad = convertToAngleInterval(angle + 120.0 - 90.0) * M_PI / 180.0;
    }
    else {
        angleFirstPointRad = convertToAngleInterval(angle + 60.0 + 90.0) * M_PI / 180.0;
        angleSecondPointRad = convertToAngleInterval(angle + 120.0 + 90.0) * M_PI / 180.0;
    }

    std::vector<Eigen::Vector3d> circlePoints {};
    Eigen::Vector3d centreCircle {0, 0, 0};
    Eigen::Vector3d firstPoint {0, 0, 0};
    Eigen::Vector3d secondPoint {0, 0, 0};

    bool noIntersections { false };
    changeRadius = false;

    // Start from the second straight line
    for(int i = 1; i < parallelStraightLines->CurvesNumber() && !noIntersections; ++i) {

        // Eval intersections between i-th straight line and the polygon
        auto intersec = parallelStraightLines->Intersection(i, polygon);         

        intersectionsCounter += (intersec.size() >= 2) ? 2 : intersec.size();

        auto index {intersectionsCounter - 1};

        if(intersec.size() >= 1) {

            double abscissa { 0 };
            // Take the previous curve and evaluate the closest point w.r.t. the nearest point on the next curve (obtain the abscissa and then generate the point).
            std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i - 1]->FindClosestPoint(intersectionPoints[index - intersec.size() + 1]);
            middlePoint = parallelStraightLines->Curves()[i - 1]->At(abscissa);
          
            
            std::shared_ptr<StraightLine> line1;
            std::shared_ptr<StraightLine> line2;

            line1 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    middlePoint);

            if(previousIntersectionsCounter == 1) {
                line2 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1]);
            }
            else {
                line2 = std::make_shared<StraightLine>(intersectionPoints[index - intersec.size() - previousIntersectionsCounter + 1], 
                    intersectionPoints[index - intersec.size()]);
            }

            if(line1->Length() >= line2->Length()) {
                // std::cout << "Iteration: " << i << " caso line1->Length() >= line2->Length()" << std::endl;
                raceTrack->AddCurveBack(line1);

                double angleTest{3.14};

                circlePoints.push_back(middlePoint);

                // Do things
                centreCircle[0] = (middlePoint[0] + intersectionPoints[intersectionsCounter - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[intersectionsCounter - intersec.size()][1]) / 2;

                if(changeRadius) {  
                    firstPoint[0] = centreCircle[0] + secondRadius * std::cos(angleFirstPointRad);
                    firstPoint[1] = centreCircle[1] + secondRadius * std::sin(angleFirstPointRad);

                    secondPoint[0] = centreCircle[0] + secondRadius * std::cos(angleSecondPointRad);
                    secondPoint[1] = centreCircle[1] + secondRadius * std::sin(angleSecondPointRad);
                }
                else {
                    firstPoint[0] = centreCircle[0] + firstRadius * std::cos(angleFirstPointRad);
                    firstPoint[1] = centreCircle[1] + firstRadius * std::sin(angleFirstPointRad);

                    secondPoint[0] = centreCircle[0] + firstRadius * std::cos(angleSecondPointRad);
                    secondPoint[1] = centreCircle[1] + firstRadius * std::sin(angleSecondPointRad);
                }
               

                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();

                std::shared_ptr<StraightLine> lineThroughBoth;

                if(changeRadius) {
                    lineThroughBoth =  std::make_shared<StraightLine>(
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * secondRadius,
                        centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * secondRadius, 0},
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * secondRadius, 
                        centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * secondRadius, 0});
                }
                else {
                    lineThroughBoth =  std::make_shared<StraightLine>(
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * firstRadius,
                        centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * firstRadius, 0},
                    Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * firstRadius, 
                        centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * firstRadius, 0});
                }

                changeRadius = !changeRadius; 

                auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(raceTrack->Curves()[raceTrack->CurvesNumber() - 1]);
                std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
                if(raceTrack->CurvesNumber() > 1)
                    lineIntersectSerpentine2 = lineThroughBoth->Intersection(raceTrack->Curves()[raceTrack->CurvesNumber() - 2]);

                if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

                    angleTest = - angleTest;

                }


                if(!changeRadius)
                    angleTest = - angleTest;

                raceTrack->AddCurveBack(std::make_shared<CircularArc>(angleTest, Eigen::Vector3d{0, 0, 1}, middlePoint, centreCircle));


            }
            else {
                // std::cout << "Iteration: " << i << " caso line1->Length() < line2->Length()" << std::endl;

                raceTrack->AddCurveBack(line2);

                double angleTest {3.14};

                std::tie(abscissa, std::ignore) = parallelStraightLines->Curves()[i]->FindClosestPoint(intersectionPoints[index - intersec.size()]);
                middlePoint = parallelStraightLines->Curves()[i]->At(abscissa);

                circlePoints.push_back(intersectionPoints[index - intersec.size()]);

                // Do things
                centreCircle[0] = (middlePoint[0] + intersectionPoints[index - intersec.size()][0]) / 2;
                centreCircle[1] = (middlePoint[1] + intersectionPoints[index - intersec.size()][1]) / 2;

                if(changeRadius) {  
                    firstPoint[0] = centreCircle[0] + secondRadius * std::cos(angleFirstPointRad);
                    firstPoint[1] = centreCircle[1] + secondRadius * std::sin(angleFirstPointRad);

                    secondPoint[0] = centreCircle[0] + secondRadius * std::cos(angleSecondPointRad);
                    secondPoint[1] = centreCircle[1] + secondRadius * std::sin(angleSecondPointRad);
                }
                else {
                    firstPoint[0] = centreCircle[0] + firstRadius * std::cos(angleFirstPointRad);
                    firstPoint[1] = centreCircle[1] + firstRadius * std::sin(angleFirstPointRad);

                    secondPoint[0] = centreCircle[0] + firstRadius * std::cos(angleSecondPointRad);
                    secondPoint[1] = centreCircle[1] + firstRadius * std::sin(angleSecondPointRad);
                }


                auto directionFirstToSecond = Eigen::Vector3d(firstPoint[0] - secondPoint[0], firstPoint[1] - secondPoint[1], 0);
                directionFirstToSecond /= directionFirstToSecond.norm();

                std::shared_ptr<StraightLine> lineThroughBoth;

                if(changeRadius) {
                    lineThroughBoth =  std::make_shared<StraightLine>(
                        Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * secondRadius,
                            centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * secondRadius, 0},
                        Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * secondRadius, 
                            centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * secondRadius, 0});
                }
                else {
                    lineThroughBoth =  std::make_shared<StraightLine>(
                        Eigen::Vector3d{centreCircle[0] + std::cos(angleFirstPointRad) - directionFirstToSecond[0] * 2 * firstRadius,
                            centreCircle[1] + std::sin(angleFirstPointRad) - directionFirstToSecond[1] * 2 * firstRadius, 0},
                        Eigen::Vector3d{centreCircle[0] + std::cos(angleSecondPointRad) + directionFirstToSecond[0] * 2 * firstRadius, 
                            centreCircle[1] + std::sin(angleSecondPointRad) + directionFirstToSecond[1] * 2 * firstRadius, 0});
                }

                changeRadius = !changeRadius; 
             

                auto lineIntersectSerpentine1 = lineThroughBoth->Intersection(raceTrack->Curves()[raceTrack->CurvesNumber() - 1]);
                std::vector<Eigen::Vector3d> lineIntersectSerpentine2{};
                if(raceTrack->CurvesNumber() > 1)
                    lineIntersectSerpentine2 = lineThroughBoth->Intersection(raceTrack->Curves()[raceTrack->CurvesNumber() - 2]);

                if(!lineIntersectSerpentine1.empty() or !lineIntersectSerpentine2.empty()) {

                    angleTest = - angleTest;

                }

                // circlePoints.push_back(firstPoint);
                // circlePoints.push_back(secondPoint);

                // circlePoints.push_back(middlePoint);

                if(!changeRadius)
                    angleTest = - angleTest;

                raceTrack->AddCurveBack(std::make_shared<CircularArc>(angleTest, Eigen::Vector3d{0, 0, 1}, intersectionPoints[index - intersec.size()], centreCircle));


            }
        }
        else {
            // std::cout << "Iteration: " << i << " -> Not anymore intersections!!" << std::endl;

            raceTrack->AddCurveBack(std::make_shared<StraightLine>(intersectionPoints[index - 1], intersectionPoints[index]));
            noIntersections = true;
        }

        circlePoints.clear();
        previousIntersectionsCounter = intersec.size();
    }
    
      return raceTrack;
}



