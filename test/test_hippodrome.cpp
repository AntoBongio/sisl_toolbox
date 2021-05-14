#include "test/test_hippodrome.hpp"


int main(int argc, char** argv) {

    auto start = std::chrono::high_resolution_clock::now();
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    std::vector<Parameters> curveDefinition1 {
        {Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{10, 0, 0}}, // Line
        {3.14, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{10, 0, 0}, Eigen::Vector3d{10, 5, 0}}, // Circle
        {Eigen::Vector3d{10, 10, 0}, Eigen::Vector3d{0, 10, 0}}, // Line
        {3.14, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{0, 10, 0}, Eigen::Vector3d{0, 5, 0}} // Circle
    };    

    auto firstPath = std::make_shared<Path>(curveDefinition1);

    auto end = std::chrono::high_resolution_clock::now();
    // Calculating total time taken by the program.
    double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    time_taken *= 1e-9;  
    std::cout << "Time taken to build the Path object : " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;


    firstPath->SavePath(80, "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

    Eigen::Vector3d findNearThis{8, 8, 3};
    double abscissaClosest{0};
    int curveIdClosest{0};
    auto closestPoint = firstPath->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);
    std::cout << "Closest point: [" << std::setprecision(5) 
              << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" 
              << std::endl;
    
    std::ofstream outputFile;
    outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
    outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
    outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
    outputFile.close();

    return 0;
}