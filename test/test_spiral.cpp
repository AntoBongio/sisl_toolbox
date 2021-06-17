#include "test/test_path.hpp"
#include <vector>

#include <iomanip>


int main() {

    /***************** Path creation *****************/

    
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    std::shared_ptr<Path> spiral;

    try {
        spiral = PathFactory::NewSpiral(Eigen::Vector3d{10, 20, 0}, Eigen::Vector3d{45, 40, 0}, 2);

        std::cout << *spiral << std::endl;

        // std::cout << *(*spiral)[0] << std::endl;

        PersistenceManager::SaveObj(spiral->Sampling(500), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

        double abscissaCurve_m{0};
        int curveId{0};

        // std::cout << std::endl << spiral->Name() << " is composed by: " << std::endl;
        // for(int i = 0; i < spiral->CurvesNumber(); ++i) {
        //     std::cout << i << ". " << *spiral->Curves()[i] << std::endl;
        // }


        /***************** Parametrizations mapping *****************/
        
        double absPath_m{10};
        std::tie(abscissaCurve_m, curveId) = spiral->PathAbsToCurveAbs(absPath_m);
        std::cout << std::endl << "Given abscissa path " << absPath_m << ", convert in Curve parametrization -> curveId: " 
            << curveId << ", abscissaCurve_m: " << abscissaCurve_m << std::endl;
        absPath_m = spiral->CurveAbsToPathAbs(abscissaCurve_m, curveId);
        std::cout << "Inverse transformation -> Given curveId " << curveId << " and abscissaCurve_m: " << abscissaCurve_m 
            << " -> , abscissa path: " << absPath_m << std::endl;


        /***************** Intersection Problem  *****************/

        std::ofstream outputIntersection;
        outputIntersection.open ("/home/antonino/Desktop/sisl_toolbox/script/intersectionPoints.txt");
        auto intersectingCurve = CurveFactory::NewCurve<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{60, 37, 0}, Eigen::Vector3d{45, 35, 0});
        auto intersectingPath = std::make_shared<Path>();
        intersectingPath->AddCurveBack(intersectingCurve);
        PersistenceManager::SaveObj(intersectingCurve->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/intersectingCurve.txt");

        auto intersectionPoints = spiral->Intersection(intersectingPath);

        std::cout << std::endl << "Given -> " << *intersectingCurve << std::endl;
        std::cout << "The intersection points are:  "<< std::endl;

        double counter{1};
        for(auto const & point: intersectionPoints) {
            std::cout << "[" << point[0] << ", " << point[1] << ", " << point[2] << "]" << std::endl;
            outputIntersection << point[0] << " " << point[1] << " " << point[2] << " " << counter++ << "\n";
        }
        outputIntersection.close();

        /***************** Extract Path Section Problem  *****************/

        auto pathSection = spiral->ExtractSection(300, 700);
        PersistenceManager::SaveObj(pathSection->Sampling(200), "/home/antonino/Desktop/sisl_toolbox/script/pathSection.txt");

        /***************** Closest Point Problem  *****************/

        Eigen::Vector3d findNearThis{-20, -20, 0};
        double abscissaClosest{0};
        int curveIdClosest{0};
        auto closestPoint = pathSection->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);

        std::cout << std::endl << "Starting from point [" << findNearThis[0] << ", " << findNearThis[1] << ", " << findNearThis[2] << "]"
            << " the closest point on the path is [" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" << std::endl;
        
        std::ofstream outputFile;
        outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
        outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
        outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
        outputFile.close();

        /***************** Extract and EvalPoint *****************/

        auto start = std::chrono::high_resolution_clock::now();

        auto pathSectionTimer = spiral->ExtractSection(300, 700);
        auto closestPointTimer = pathSectionTimer->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);

        auto end = std::chrono::high_resolution_clock::now();
        // Calculating total time taken by the program.
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        time_taken *= 1e-9;  
        std::cout << "Time taken to Extract and EvalPoint : " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;
        std::cout << std::fixed << std::setprecision(3); 
            
        /***************** Move Point Problem  *****************/

        double abscissaStartPoint{10};
        double offset{30};
        std::ofstream outputFile2;
        outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt"); 

        auto startPoint = spiral->At(abscissaStartPoint);
        std::cout << std::endl << "Point at abscissa: " << abscissaStartPoint << " is [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";
        
        abscissaStartPoint += offset;
        startPoint = spiral->At(abscissaStartPoint);
        std::cout << "Moved at abscissa: " << abscissaStartPoint << " -> [" << startPoint[0] << ", " 
            << startPoint[1] << ", " << startPoint[2] << "]" << std::endl;
        outputFile2 << abscissaStartPoint << " " << startPoint[0] << " " << startPoint[1] << " " << startPoint[2] << "\n";

        outputFile2.close();

        /***************** Test Derivatives  *****************/

        auto derivativesCurve = spiral->Curves()[0]->Derivate(3, 20);
        auto derivativesPath = spiral->Derivate(3, 30000);

        
    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }

    return 0;
}