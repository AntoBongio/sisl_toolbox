#include "test/test_serpentine.hpp"


int main(int argc, char** argv) {

    /** Save start time */
    auto start = std::chrono::high_resolution_clock::now();
    /** unsync the I/O of C and C++.
     * It is often recommended to use scanf/printf instead of cin/cout for a fast input and output. 
     * However, you can still use cin/cout and achieve the same speed as scanf/printf by including the following line.
     * It toggles on or off the synchronization of all the C++ standard streams with their corresponding standard C streams 
     * if it is called before the program performs its first input or output operation.
     */
    std::ios_base::sync_with_stdio(false);
    
    
    std::vector<Parameters> curveDefinition;

    std::string pathJson = "/home/antonino/Desktop/sisl_toolbox/test/path.json";

    Json::Value root, curveRoot;
    Json::Reader reader;

    std::ifstream file(pathJson);
    file >> root;
    reader.parse(file, root, true);    

    ctb::LatLong centroid{root["centroid"][0].asDouble(), 
                          root["centroid"][1].asDouble()};
    bool reverse = root["direction"].asInt() ? true : false;     
    
    for(auto curve: root["curves"]) {

        reader.parse(curve.toStyledString(), curveRoot); // provare senza toStyledString

        int degree {curveRoot["degree"].asInt()}; 

        std::vector<double> weights;
        for(auto i = 0; i < curveRoot["weigths"].size(); i++) {
            weights.push_back(curveRoot["weigths"][i].asDouble());
        }

        std::vector<Eigen::Vector3d> points(curveRoot["points"].size(), Eigen::Vector3d::Zero());
        for(auto i = 0; i < curveRoot["points"].size(); i++) {
            ctb::LatLong pointLatLong;
            pointLatLong.latitude = curveRoot["points"][i][0].asDouble();
            pointLatLong.longitude = curveRoot["points"][i][1].asDouble();
            ctb::LatLong2LocalUTM(pointLatLong, 0.0, centroid, points[i]);
        }

        std::vector<double> knots;
        for(auto i = 0; i < curveRoot["knots"].size(); i++) {
            knots.push_back(curveRoot["knots"][i].asDouble());
        }

        curveDefinition.emplace_back(degree, knots, points, weights);
    }

    auto firstPath = std::make_shared<Path>(curveDefinition);

    /** Save end time */
    auto end = std::chrono::high_resolution_clock::now();
    // Calculating total time taken by the program.
    double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() * 1e-9;
    /** std::fixed -> fixed-point notation / std::precision(n) -> from this point n decimals precision */
    std::cout << "Time taken to build the Path object: " << std::fixed << std::setprecision(9) << time_taken  << " sec" << std::endl;
    
    std::cout << "Length: " << std::setprecision(3) << firstPath->Length() << std::endl; 

    firstPath->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/path.txt");
    
    Eigen::Vector3d findNearThis{-130, 0, 3};
    double abscissaClosest{0};
    int curveIdClosest{0};
    auto closestPoint = firstPath->FindClosestPoint(findNearThis, curveIdClosest, abscissaClosest);
    std::cout << "curveIdClosest: " << curveIdClosest << ", abscissaClosest: " << abscissaClosest << std::endl;
    std::cout << std::setprecision(5) 
              << "Closest point to [" << findNearThis[0] << ", " << findNearThis[1] << ", " << findNearThis[2] << " ] is: [" 
              << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << "]" 
              << std::endl;
        
    std::ofstream outputFile;
    outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/closestPoint.txt");
    outputFile << "FindNear " << findNearThis[0] << " " << findNearThis[1] << " " << findNearThis[2] << "\n";
    outputFile << "ClosestPoint " << closestPoint[0] << " " << closestPoint[1] << " " << closestPoint[2] << "\n";
    outputFile.close();

    return 0;
}