#include "test/test_serpentine.hpp"
#include <vector>

#include <iomanip>


int main() {

    /** TEST: Circle test */
    // try {
        
    //     auto circle = std::make_shared<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{10, 10, 0}, Eigen::Vector3d{2, 2, 1});

    //     PersistenceManager::SaveObj(circle->Sampling(1500), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");
    //     std::cout << "Circle curvature at 15 -> " << circle->Curvature(15) << std::endl;

    //     std::cout << *circle << std::endl;

    //     /***************** Intersection Problem  *****************/

    //     std::ofstream outputIntersection;
    //     outputIntersection.open ("/home/antonino/Desktop/sisl_toolbox/script/intersectionPoints.txt");
    //     auto intersectingCurve = CurveFactory::NewCurve<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{8, 8, 0}, Eigen::Vector3d{12, 12, 0});
    //     PersistenceManager::SaveObj(intersectingCurve->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/intersectingCurve.txt");

    //     auto intersectionPoints = circle->Intersection(intersectingCurve);

    //     std::cout << std::endl << "Given -> " << *intersectingCurve << std::endl;
    //     std::cout << "The intersection points are:  "<< std::endl;

    //     double counter{1};
    //     for(auto const & point: intersectionPoints) {
    //         std::cout << "[" << point[0] << ", " << point[1] << ", " << point[2] << "]" << std::endl;
    //         outputIntersection << point[0] << " " << point[1] << " " << point[2] << " " << counter++ << "\n";
    //     }
    //     outputIntersection.close();


    //     /***************** Closest Point Problem  *****************/

    //     Eigen::Vector3d findNearThis{-10, -10, 0};
    //     double abscissaClosest{0};
    //     double distance{0};
    //     int curveIdClosest{0};

    //     try {
    //         std::tie(abscissaClosest, distance) = circle->FindClosestPoint(findNearThis);
    //     } 
    //     catch (std::runtime_error const& exception) {
    //         std::cout << "Exception -> " << exception.what() << std::endl;
    //     }
    //     Eigen::Vector3d closestPoint{circle->At(abscissaClosest)};

    //     std::cout << std::endl << "Starting from point [" << findNearThis[0] << ", " << findNearThis[1] << ", " << findNearThis[2] << "]"
    //         << " the closest point on the path is [" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" << std::endl;
        
    //     std::ofstream outputFile;
    //     outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
    //     outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
    //     outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
    //     outputFile.close();


    //     /***************** Move Point Problem  *****************/

    //     double abscissaStartPoint{3};
    //     double offset{10};
    //     std::ofstream outputFile2;
    //     outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt"); 

    //     auto startPoint = circle->At(abscissaStartPoint);
    //     std::cout << std::endl << "Point at abscissa: " << abscissaStartPoint << " is [" << startPoint[0] << ", " 
    //         << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
    //     outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";
        
    //     abscissaStartPoint += offset;
    //     startPoint = circle->At(abscissaStartPoint);
    //     std::cout << "Moved at abscissa: " << abscissaStartPoint << " -> [" << startPoint[0] << ", " 
    //         << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
    //     outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";

    //     outputFile2.close();


    //     /***************** Extract Path Section Problem  *****************/

    //     std::shared_ptr<Curve> circleSection;
    //     try {
    //         circleSection = circle->ExtractSection(2, 20);
    //     } 
    //     catch (std::runtime_error const& exception) {
    //         std::cout << "Exception -> " << exception.what() << std::endl;
    //     }
            
    //     if(circleSection)
    //         PersistenceManager::SaveObj(circleSection->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/pathSection.txt");
    // }
    // catch(std::runtime_error const& exception) {
    //     std::cout << "Received exception from --> " << exception.what() << std::endl;
    // }


    /***************** Path creation *****************/
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    ctb::LatLong centroid{44.39173292288923, 8.945241571195552};

    std::vector<ctb::LatLong> pointLatLong {
        ctb::LatLong(44.39103097698746, 8.94579379703305) , 
        ctb::LatLong(44.39130994954367, 8.946484085232179) ,
        ctb::LatLong(44.3921468857443, 8.946245637251524) , 
        ctb::LatLong(44.39244386538219, 8.94470188296125) ,
        ctb::LatLong(44.3915259324859, 8.943999052589279) ,
        ctb::LatLong(44.39074299756334, 8.94435048387282)};


    std::vector<Eigen::Vector3d> polygonVerteces(pointLatLong.size(), Eigen::Vector3d::Zero());
    for(std::size_t i = 0; i < polygonVerteces.size(); i++) {
        ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygonVerteces[i]);
        polygonVerteces[i][2] = 0;
    }

    double angle{150.0}; 
    double offsetPath{30.0};

    std::shared_ptr<Path> serpentine;

    try {
        auto start = std::chrono::high_resolution_clock::now();
        serpentine = PathFactory::NewSerpentine(angle, offsetPath, polygonVerteces);
        auto end = std::chrono::high_resolution_clock::now();

        std::cout << *serpentine << std::endl;

        // Calculating total time taken by the program.
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        time_taken *= 1e-9;  
        std::cout << "Time taken to build the Path object : " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;
        std::cout << std::fixed << std::setprecision(3); 

        PersistenceManager::SaveObj(serpentine->Sampling(1500), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

        double abscissaCurve_m{0};
        int curveId{0};

        // std::cout << std::endl << serpentine->Name() << " is composed by: " << std::endl;
        // for(int i = 0; i < serpentine->CurvesNumber(); ++i) {
        //     std::cout << i << ". " << *serpentine->Curves()[i] << std::endl;
        // }


        /***************** Parametrizations mapping *****************/
        
        double absPath_m{10};
        std::tie(abscissaCurve_m, curveId) = serpentine->PathAbsToCurveAbs(absPath_m);
        std::cout << std::endl << "Given abscissa path " << absPath_m << ", convert in Curve parametrization -> curveId: " 
            << curveId << ", abscissaCurve_m: " << abscissaCurve_m << std::endl;
        absPath_m = serpentine->CurveAbsToPathAbs(abscissaCurve_m, curveId);
        std::cout << "Inverse transformation -> Given curveId " << curveId << " and abscissaCurve_m: " << abscissaCurve_m 
            << " -> , abscissa path: " << absPath_m << std::endl;


        /***************** Intersection Problem  *****************/

        std::ofstream outputIntersection;
        outputIntersection.open ("/home/antonino/Desktop/sisl_toolbox/script/intersectionPoints.txt");
        auto intersectingCurve = CurveFactory::NewCurve<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{60, 37, 0}, Eigen::Vector3d{45, 35, 0});
        auto intersectingPath = std::make_shared<Path>();
        intersectingPath->AddCurveBack(intersectingCurve);
        PersistenceManager::SaveObj(intersectingCurve->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/intersectingCurve.txt");

        auto intersectionPoints = serpentine->Intersection(intersectingPath);

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
            closestPoint = serpentine->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);
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

        auto startPoint = serpentine->At(abscissaStartPoint);
        std::cout << std::endl << "Point at abscissa: " << abscissaStartPoint << " is [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";
        
        abscissaStartPoint += offset;
        startPoint = serpentine->At(abscissaStartPoint);
        std::cout << "Moved at abscissa: " << abscissaStartPoint << " -> [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";

        outputFile2.close();


        /***************** Extract Path Section Problem  *****************/

        std::shared_ptr<Path> pathSection;
        try {
            pathSection = serpentine->ExtractSection(300, 700);
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
            derivatives = serpentine->Derivate(2, 300);
            curvature = serpentine->Curvature(400);
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