#include "test/test_serpentine.hpp"
#include <vector>

#include <iomanip>


int main(int argc, char** argv) {

    /***************** Path creation *****************/

    auto start = std::chrono::high_resolution_clock::now();
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
    for(auto i = 0; i < polygonVerteces.size(); i++) {
        ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygonVerteces[i]);
        polygonVerteces[i][2] = 0;
    }

    double angle{90.0}; 
    double offsetPath{10.0};

    std::shared_ptr<Path> serpentine;

    try {
        serpentine = PathFactory::NewSerpentine(angle, offsetPath, polygonVerteces);

        std::cout << *serpentine << std::endl;

        auto end = std::chrono::high_resolution_clock::now();
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
        std::tie(abscissaCurve_m, curveId, std::ignore) = serpentine->PathAbsToCurveAbs(absPath_m);
        std::cout << std::endl << "Given abscissa path " << absPath_m << ", convert in Curve parametrization -> curveId: " 
            << curveId << ", abscissaCurve_m: " << abscissaCurve_m << std::endl;
        std::tie(absPath_m, std::ignore) = serpentine->CurveAbsToPathAbs(abscissaCurve_m, curveId);
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

        Eigen::Vector3d findNearThis{-20, -20, 0};
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
    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }

    return 0;
}