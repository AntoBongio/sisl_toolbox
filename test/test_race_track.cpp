#include "test/test_serpentine.hpp"
#include <vector>

#include <iomanip>


int main() {


    /***************** Path creation *****************/
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    std::vector<Eigen::Vector3d> polygonVerteces {
        Eigen::Vector3d {-78, 44, 0}, Eigen::Vector3d {-47, 99, 0}, Eigen::Vector3d {46, 80, 0}, 
        Eigen::Vector3d {79, -43, 0}, Eigen::Vector3d {-23, -99, 0}, Eigen::Vector3d{-110, -71, 0} };


    double angle{360.0}; 
    double offsetPath{20.0};
    double firstRadius {36.0};
    double secondRadius {17.0};
    int direction{RIGHT};

    std::shared_ptr<Path> raceTrack;

    try {
        auto start = std::chrono::high_resolution_clock::now();
        raceTrack = PathFactory::NewRaceTrack(angle, direction,  firstRadius, secondRadius, polygonVerteces);
        auto end = std::chrono::high_resolution_clock::now();

        auto polygon = PathFactory::NewPolygon(polygonVerteces);
        PersistenceManager::SaveObj(polygon->Sampling(100), "/home/antonino/Desktop/sisl_toolbox/script/polygon.txt");

        std::cout << *raceTrack << std::endl;

        // Calculating total time taken by the program.
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        time_taken *= 1e-9;  
        std::cout << "Time taken to build the Path object : " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;
        std::cout << std::fixed << std::setprecision(3); 

        PersistenceManager::SaveObj(raceTrack->Sampling(1500), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

        double abscissaCurve_m{0};
        int curveId{0};

        // std::cout << std::endl << raceTrack->Name() << " is composed by: " << std::endl;
        // for(int i = 0; i < raceTrack->CurvesNumber(); ++i) {
        //     std::cout << i << ". " << *raceTrack->Curves()[i] << std::endl;
        // }


        /***************** Parametrizations mapping *****************/
        
        double absPath_m{10};
        std::tie(abscissaCurve_m, curveId) = raceTrack->PathAbsToCurveAbs(absPath_m);
        std::cout << std::endl << "Given abscissa path " << absPath_m << ", convert in Curve parametrization -> curveId: " 
            << curveId << ", abscissaCurve_m: " << abscissaCurve_m << std::endl;
        absPath_m = raceTrack->CurveAbsToPathAbs(abscissaCurve_m, curveId);
        std::cout << "Inverse transformation -> Given curveId " << curveId << " and abscissaCurve_m: " << abscissaCurve_m 
            << " -> , abscissa path: " << absPath_m << std::endl;


        /***************** Intersection Problem  *****************/

        std::ofstream outputIntersection;
        outputIntersection.open ("/home/antonino/Desktop/sisl_toolbox/script/intersectionPoints.txt");
        auto intersectingCurve = std::make_shared<CircularArc>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{60, 37, 0}, Eigen::Vector3d{45, 35, 0});
        auto intersectingPath = std::make_shared<Path>();
        intersectingPath->AddCurveBack(intersectingCurve);
        PersistenceManager::SaveObj(intersectingCurve->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/intersectingCurve.txt");

        auto intersectionPoints = raceTrack->Intersection(intersectingPath);

        std::cout << std::endl << "Given -> " << *intersectingCurve << std::endl;
        std::cout << "The intersection points are:  "<< std::endl;

        double counter{1};
        for(auto const & point: intersectionPoints) {
            std::cout << "[" << point[0] << ", " << point[1] << ", " << point[2] << "]" << std::endl;
            outputIntersection << point[0] << " " << point[1] << " " << point[2] << " " << counter++ << "\n";
        }
        outputIntersection.close();


        /***************** Closest Point Problem  *****************/

        Eigen::Vector3d findNearThis{12, -120, 0};
        double abscissaClosest{0};
        int curveIdClosest{0};

        Eigen::Vector3d closestPoint{};
        try {
            closestPoint = raceTrack->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);
        } 
        catch (std::runtime_error const& exception) {
            std::cout << "Exception -> " << exception.what() << std::endl;
        }

        std::cout << std::endl << "Starting from point [" << findNearThis[0] << ", " << findNearThis[1] << ", " << findNearThis[2] << "]"
            << " the closest point on the path is [" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" << std::endl;
        
        std::ofstream outputFile;
        outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
        outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
        outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
        outputFile.close();


        /***************** Move Point Problem  *****************/

        double abscissaStartPoint{10};
        double offset{30};
        std::ofstream outputFile2;
        outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt"); 

        auto startPoint = raceTrack->At(abscissaStartPoint);
        std::cout << std::endl << "Point at abscissa: " << abscissaStartPoint << " is [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";
        
        abscissaStartPoint += offset;
        startPoint = raceTrack->At(abscissaStartPoint);
        std::cout << "Moved at abscissa: " << abscissaStartPoint << " -> [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";

        outputFile2.close();


        /***************** Extract Path Section Problem  *****************/

        std::shared_ptr<Path> pathSection;
        try {
            pathSection = raceTrack->ExtractSection(300, 700);
        } 
        catch (std::runtime_error const& exception) {
            std::cout << "Exception -> " << exception.what() << std::endl;
        }
            
        if(pathSection)
            PersistenceManager::SaveObj(pathSection->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/pathSection.txt");

        /***************** Test Derivate and Curvature *****************/

        
        std::vector<Eigen::Vector3d> derivatives;
        double curvature;
        try {
            derivatives = raceTrack->Derivate(2, 300);
            curvature = raceTrack->Curvature(400);
        } 
        catch (std::runtime_error const& exception) {
            std::cout << "Exception -> " << exception.what() << std::endl;
        }
        
        std::cout << derivatives.size() << std::endl;
        std::cout << "Curvature at 400m: [" << curvature << "]" << std::endl;  
        std::cout << "First order derivative at 300m: [" << derivatives[0][0] << ", " << derivatives[0][1] << ", " << derivatives[0][2] << "]" << std::endl;
        std::cout << "Second order derivative at 300m: [" << derivatives[1][0] << ", " << derivatives[1][1] << ", " << derivatives[1][2] << "]" << std::endl;


    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }

    return 0;
}