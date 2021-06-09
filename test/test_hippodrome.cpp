#include "test/test_hippodrome.hpp"
#include <vector>

#include <iomanip>


int main(int argc, char** argv) {

    auto start = std::chrono::high_resolution_clock::now();
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    std::shared_ptr<Path> hippodrome;

    try {
        hippodrome = PathFactory::NewHippodrome(std::vector<Eigen::Vector3d>{Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{10, 0, 0}, 
                                                                            Eigen::Vector3d{10, 10, 0}, Eigen::Vector3d{0, 10, 0}});
    }
    catch(std::string const & exception) {
        std::cout << "[test_hippodrome main] received exception from --> " << exception << std::endl;
        return 0;
    }

    auto end = std::chrono::high_resolution_clock::now();
    // Calculating total time taken by the program.
    double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    time_taken *= 1e-9;  
    std::cout << "Time taken to build the Path object : " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;

    PersistenceManager::SaveObj(hippodrome->Sampling(100), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

    std::cout << "Hippodrome->Length(): " << hippodrome->Length() << std::endl;

    overBound overBound{};
    double abscissaCurve_m{0};
    int curveId{0};

    for(int i = 0; i < hippodrome->CurvesNumber(); ++i) {
        std::cout << "Curve " << i << " -> Length() " << hippodrome->Curves()[i]->Length() << std::endl;
    }
    
    std::tie(abscissaCurve_m, curveId, overBound) = hippodrome->PathAbsToCurveAbs(45);

    std::cout << "curveId: " << curveId << ", abscissaCurve_m: " << abscissaCurve_m << std::endl;

    // Eigen::Vector3d findNearThis{15.5, 13, 0};
    // double abscissaClosest{0};
    // int curveIdClosest{0};
    // auto closestPoint = hippodrome->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);
    // std::cout << "Closest point: [" << std::setprecision(5) 
    //           << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" 
    //           << std::endl;
    
    // std::ofstream outputFile;
    // outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
    // outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
    // outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
    // outputFile.close();

    return 0;
}